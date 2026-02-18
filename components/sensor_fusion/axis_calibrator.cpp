/**
 * @file axis_calibrator.cpp
 * @brief Auto-calibration of IMU axis orientation using GPS velocity vectors
 *
 * Solves Wahba's problem: given paired observations of the same vector
 * in two coordinate frames (sensor and NED), find the rotation matrix R
 * that maps sensor frame → NED frame.
 *
 * Method:
 *   1. Differentiate GPS NED velocity → aNED (acceleration in NED frame)
 *   2. Read IMU accel (gravity-removed by boot calibration) → aSensor
 *   3. Collect paired samples (aNED, aSensor) during driving
 *   4. Build cross-covariance H = Σ aSensor · aNED^T
 *   5. SVD(H) = U·S·V^T → R = V·diag(1,1,d)·U^T where d = det(V)·det(U)
 *
 * Persistence: saves/loads R to /spiffs/config/imu_fusion.json via cJSON
 */

#include "sensor_fusion.h"

#include <cmath>
#include <cstring>
#include <cstdio>

#include "esp_log.h"
#include "cJSON.h"

static const char* TAG = "AxisCalib";

// ============================================================
// CONFIGURATION
// ============================================================

static constexpr int    CALIB_MIN_SAMPLES     = 50;
static constexpr int    CALIB_MAX_SAMPLES     = 200;
static constexpr float  CALIB_MIN_SPEED_KMH   = 15.0f;   // GPS speed gate
static constexpr float  CALIB_MIN_ACCEL_MAG   = 0.3f;     // m/s^2 minimum dynamic
static constexpr float  CALIB_MAX_ACCEL_MAG   = 8.0f;     // m/s^2 saturation guard
static constexpr float  CALIB_MAG_RATIO_MIN   = 0.3f;     // |aNED|/|aSensor| min
static constexpr float  CALIB_MAG_RATIO_MAX   = 3.0f;     // |aNED|/|aSensor| max
static constexpr float  CALIB_MIN_DT          = 0.05f;    // GPS frame dt min (s)
static constexpr float  CALIB_MAX_DT          = 0.5f;     // GPS frame dt max (s)
static constexpr float  CALIB_RESIDUAL_THRESH = 1.0f;     // RMS residual threshold (m/s^2)
static constexpr float  G_MPS2               = 9.80665f;  // gravity (m/s^2)
static constexpr int    SVD_SWEEPS            = 8;         // Jacobi SVD iterations

static const char* CALIB_PATH = "/spiffs/config/imu_fusion.json";

// ============================================================
// SAMPLE STORAGE
// ============================================================

struct CalibSample {
    float aNED[3];      // GPS-derived acceleration (m/s^2, NED frame)
    float aSensor[3];   // IMU acceleration (m/s^2, sensor frame)
};

static CalibSample s_samples[CALIB_MAX_SAMPLES];
static int s_sampleCount = 0;
static int s_sampleWriteIdx = 0;  // circular buffer write index

// Previous GPS velocity for differentiation
static float s_prevVelN = 0.0f, s_prevVelE = 0.0f, s_prevVelD = 0.0f;
static uint32_t s_prevITOW = 0;
static bool s_hasPrev = false;

// ============================================================
// CALIBRATION RESULT
// ============================================================

static float s_R[9] = {1,0,0, 0,1,0, 0,0,1};  // identity default
static float s_residualRMS = -1.0f;
static int   s_calibSamplesUsed = 0;
static bool  s_calibValid = false;

// ============================================================
// 3x3 MATRIX UTILITIES
// ============================================================

static inline float vec3Dot(const float a[3], const float b[3])
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

static inline float vec3Mag(const float v[3])
{
    return sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

// C = A * B (3x3, row-major)
static void mat3Mul(const float A[9], const float B[9], float C[9])
{
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            C[i*3+j] = A[i*3+0]*B[0*3+j] + A[i*3+1]*B[1*3+j] + A[i*3+2]*B[2*3+j];
        }
    }
}

// C = A^T (3x3, row-major)
static void mat3Transpose(const float A[9], float AT[9])
{
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            AT[i*3+j] = A[j*3+i];
}

// determinant of 3x3 row-major
static float mat3Det(const float M[9])
{
    return M[0]*(M[4]*M[8] - M[5]*M[7])
         - M[1]*(M[3]*M[8] - M[5]*M[6])
         + M[2]*(M[3]*M[7] - M[4]*M[6]);
}

// ============================================================
// JACOBI SVD FOR 3x3 MATRICES
// ============================================================

// Givens rotation parameters for a 2x2 symmetric matrix
// [a c; c b] → cos/sin that diagonalizes it
static void givens2x2Sym(float a, float b, float c, float* cs, float* sn)
{
    if (fabsf(c) < 1e-10f) {
        *cs = 1.0f;
        *sn = 0.0f;
        return;
    }
    float tau = (b - a) / (2.0f * c);
    float t;
    if (tau >= 0.0f) {
        t = 1.0f / (tau + sqrtf(1.0f + tau * tau));
    } else {
        t = -1.0f / (-tau + sqrtf(1.0f + tau * tau));
    }
    *cs = 1.0f / sqrtf(1.0f + t * t);
    *sn = t * (*cs);
}

/**
 * @brief One-sided Jacobi SVD for a 3x3 matrix.
 *
 * Computes B = U · S · V^T where B is the input 3x3 matrix (row-major).
 * U and V are 3x3 orthogonal matrices (row-major).
 * S (singular values) stored in sigma[3].
 *
 * Uses repeated Jacobi rotations on B^T·B to find V, then U = B·V·S^{-1}.
 */
static void svd3x3(const float B[9], float U[9], float sigma[3], float V[9])
{
    // Work on a copy
    float A[9];
    memcpy(A, B, sizeof(A));

    // V starts as identity
    float Vmat[9] = {1,0,0, 0,1,0, 0,0,1};

    // Jacobi sweeps: apply Givens rotations to columns of A to diagonalize A^T*A
    for (int sweep = 0; sweep < SVD_SWEEPS; sweep++) {
        // Sweep over all 3 pairs: (0,1), (0,2), (1,2)
        for (int p = 0; p < 2; p++) {
            for (int q = p + 1; q < 3; q++) {
                // Compute elements of A^T*A for the (p,q) 2x2 subproblem
                float app = 0, aqq = 0, apq = 0;
                for (int k = 0; k < 3; k++) {
                    app += A[k*3+p] * A[k*3+p];
                    aqq += A[k*3+q] * A[k*3+q];
                    apq += A[k*3+p] * A[k*3+q];
                }

                if (fabsf(apq) < 1e-10f) continue;

                float cs, sn;
                givens2x2Sym(app, aqq, apq, &cs, &sn);

                // Apply Givens rotation to columns p,q of A (right multiply)
                for (int k = 0; k < 3; k++) {
                    float tmp_p = A[k*3+p];
                    float tmp_q = A[k*3+q];
                    A[k*3+p] =  cs * tmp_p + sn * tmp_q;
                    A[k*3+q] = -sn * tmp_p + cs * tmp_q;
                }

                // Accumulate into V
                for (int k = 0; k < 3; k++) {
                    float tmp_p = Vmat[k*3+p];
                    float tmp_q = Vmat[k*3+q];
                    Vmat[k*3+p] =  cs * tmp_p + sn * tmp_q;
                    Vmat[k*3+q] = -sn * tmp_p + cs * tmp_q;
                }
            }
        }
    }

    // After Jacobi iterations, A ≈ U · S (columns of A are U·sigma_i)
    // Singular values = column norms of A
    for (int j = 0; j < 3; j++) {
        float norm = 0;
        for (int i = 0; i < 3; i++) {
            norm += A[i*3+j] * A[i*3+j];
        }
        sigma[j] = sqrtf(norm);
    }

    // U = A · diag(1/sigma)  (normalize columns)
    for (int j = 0; j < 3; j++) {
        float inv = (sigma[j] > 1e-10f) ? (1.0f / sigma[j]) : 0.0f;
        for (int i = 0; i < 3; i++) {
            U[i*3+j] = A[i*3+j] * inv;
        }
    }

    memcpy(V, Vmat, sizeof(Vmat));
}

// ============================================================
// WAHBA SOLVER
// ============================================================

/**
 * @brief Solve Wahba's problem from collected sample pairs.
 *
 * Finds R such that: aNED ≈ R · aSensor
 *
 * H = Σ aSensor_i · aNED_i^T   (cross-covariance)
 * SVD(H) = U · S · V^T
 * R = V · diag(1, 1, det(V)*det(U)) · U^T
 */
static bool solveWahba(const CalibSample* samples, int count,
                       float R_out[9], float* residualRMS)
{
    if (count < CALIB_MIN_SAMPLES) return false;

    // Build cross-covariance matrix H = Σ aSensor · aNED^T
    float H[9] = {};
    for (int n = 0; n < count; n++) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                H[i*3+j] += samples[n].aSensor[i] * samples[n].aNED[j];
            }
        }
    }

    // SVD: H = U · S · V^T
    float U[9], sigma[3], V[9];
    svd3x3(H, U, sigma, V);

    // Ensure proper rotation (det = +1)
    float d = mat3Det(V) * mat3Det(U);
    float D[9] = {1,0,0, 0,1,0, 0,0,d};

    // R = V · D · U^T
    float UT[9], DUT[9];
    mat3Transpose(U, UT);
    mat3Mul(D, UT, DUT);
    mat3Mul(V, DUT, R_out);

    // Verify det(R) ≈ +1
    float detR = mat3Det(R_out);
    if (fabsf(detR - 1.0f) > 0.1f) {
        ESP_LOGW(TAG, "R det=%.3f (expected 1.0), rejecting", detR);
        return false;
    }

    // Compute RMS residual: sqrt(1/N · Σ |aNED - R·aSensor|^2)
    float sumSqErr = 0;
    for (int n = 0; n < count; n++) {
        float predicted[3] = {};
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                predicted[i] += R_out[i*3+j] * samples[n].aSensor[j];
            }
        }
        for (int i = 0; i < 3; i++) {
            float e = samples[n].aNED[i] - predicted[i];
            sumSqErr += e * e;
        }
    }
    *residualRMS = sqrtf(sumSqErr / (float)count);

    return true;
}

// ============================================================
// SPIFFS PERSISTENCE
// ============================================================

static bool loadFromFile(void)
{
    FILE* f = fopen(CALIB_PATH, "r");
    if (!f) return false;

    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);

    if (size <= 0 || size > 2048) {
        fclose(f);
        return false;
    }

    char* buf = (char*)malloc(size + 1);
    if (!buf) { fclose(f); return false; }

    fread(buf, 1, size, f);
    fclose(f);
    buf[size] = '\0';

    cJSON* root = cJSON_Parse(buf);
    free(buf);
    if (!root) return false;

    // Parse R[9]
    cJSON* jR = cJSON_GetObjectItem(root, "R");
    if (!jR || !cJSON_IsArray(jR) || cJSON_GetArraySize(jR) != 9) {
        cJSON_Delete(root);
        return false;
    }
    float R[9];
    for (int i = 0; i < 9; i++) {
        cJSON* item = cJSON_GetArrayItem(jR, i);
        if (!item || !cJSON_IsNumber(item)) {
            cJSON_Delete(root);
            return false;
        }
        R[i] = (float)item->valuedouble;
    }

    // Parse residual and samples
    cJSON* jRes = cJSON_GetObjectItem(root, "residual");
    cJSON* jN   = cJSON_GetObjectItem(root, "samples");
    float residual = jRes ? (float)jRes->valuedouble : 999.0f;
    int nSamples   = jN   ? jN->valueint : 0;
    cJSON_Delete(root);

    // Validate: R^T · R ≈ I (orthogonality check)
    float RT[9], RTR[9];
    mat3Transpose(R, RT);
    mat3Mul(RT, R, RTR);

    float frobErr = 0;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            float expected = (i == j) ? 1.0f : 0.0f;
            float e = RTR[i*3+j] - expected;
            frobErr += e * e;
        }
    }
    if (frobErr > 0.01f) {
        ESP_LOGW(TAG, "Loaded R not orthogonal (frobErr=%.4f), discarding", frobErr);
        return false;
    }

    // Accept
    memcpy(s_R, R, sizeof(s_R));
    s_residualRMS = residual;
    s_calibSamplesUsed = nSamples;
    s_calibValid = true;

    return true;
}

static bool saveToFile(void)
{
    cJSON* root = cJSON_CreateObject();
    if (!root) return false;

    // R as flat array of 9 floats
    cJSON* jR = cJSON_CreateFloatArray(s_R, 9);
    cJSON_AddItemToObject(root, "R", jR);
    cJSON_AddNumberToObject(root, "residual", s_residualRMS);
    cJSON_AddNumberToObject(root, "samples", s_calibSamplesUsed);

    char* json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!json) return false;

    FILE* f = fopen(CALIB_PATH, "w");
    if (!f) { free(json); return false; }

    fputs(json, f);
    fclose(f);
    free(json);

    return true;
}

// ============================================================
// PUBLIC API
// ============================================================

void axisCalibInit(void)
{
    s_sampleCount = 0;
    s_sampleWriteIdx = 0;
    s_hasPrev = false;
    // Don't clear s_R / s_calibValid — those may be loaded from SPIFFS
}

void axisCalibReset(void)
{
    s_sampleCount = 0;
    s_sampleWriteIdx = 0;
    s_hasPrev = false;
    // Keep s_R and s_calibValid intact (saved calibration still good)
}

bool axisCalibFeedGPS(float velNorthMps, float velEastMps, float velDownMps,
                      float speedKmh, uint32_t iTOW,
                      const float accelSensor[3])
{
    // Need two consecutive frames to differentiate
    if (!s_hasPrev) {
        s_prevVelN = velNorthMps;
        s_prevVelE = velEastMps;
        s_prevVelD = velDownMps;
        s_prevITOW = iTOW;
        s_hasPrev = true;
        return false;
    }

    // Compute dt from iTOW (handles weekly rollover)
    int32_t dITOW = (int32_t)(iTOW - s_prevITOW);
    if (dITOW < 0) dITOW += 604800000;  // week rollover
    float dt = dITOW * 0.001f;

    // Save current as previous for next call
    float prevN = s_prevVelN, prevE = s_prevVelE, prevD = s_prevVelD;
    s_prevVelN = velNorthMps;
    s_prevVelE = velEastMps;
    s_prevVelD = velDownMps;
    s_prevITOW = iTOW;

    // dt sanity check
    if (dt < CALIB_MIN_DT || dt > CALIB_MAX_DT) return false;

    // GPS speed gate
    if (speedKmh < CALIB_MIN_SPEED_KMH) return false;

    // GPS-derived acceleration (NED, m/s^2)
    float aNED[3] = {
        (velNorthMps - prevN) / dt,
        (velEastMps  - prevE) / dt,
        (velDownMps  - prevD) / dt
    };

    float magNED = vec3Mag(aNED);
    if (magNED < CALIB_MIN_ACCEL_MAG || magNED > CALIB_MAX_ACCEL_MAG) return false;

    // IMU sensor accel in m/s^2 (input is in g)
    float aSensorMs2[3] = {
        accelSensor[0] * G_MPS2,
        accelSensor[1] * G_MPS2,
        accelSensor[2] * G_MPS2
    };

    float magSensor = vec3Mag(aSensorMs2);
    if (magSensor < 1e-6f) return false;

    // Magnitude ratio check (outlier rejection)
    float ratio = magNED / magSensor;
    if (ratio < CALIB_MAG_RATIO_MIN || ratio > CALIB_MAG_RATIO_MAX) return false;

    // Store sample (circular buffer)
    CalibSample& s = s_samples[s_sampleWriteIdx];
    s.aNED[0] = aNED[0]; s.aNED[1] = aNED[1]; s.aNED[2] = aNED[2];
    s.aSensor[0] = aSensorMs2[0]; s.aSensor[1] = aSensorMs2[1]; s.aSensor[2] = aSensorMs2[2];

    s_sampleWriteIdx = (s_sampleWriteIdx + 1) % CALIB_MAX_SAMPLES;
    if (s_sampleCount < CALIB_MAX_SAMPLES) s_sampleCount++;

    return true;
}

bool axisCalibSolve(void)
{
    if (s_sampleCount < CALIB_MIN_SAMPLES) {
        return false;
    }

    float R_new[9];
    float residual;

    if (!solveWahba(s_samples, s_sampleCount, R_new, &residual)) {
        ESP_LOGW(TAG, "Wahba solve failed (N=%d)", s_sampleCount);
        return false;
    }

    if (residual > CALIB_RESIDUAL_THRESH) {
        ESP_LOGW(TAG, "Calibration residual %.3f > threshold %.3f, rejecting",
                 residual, CALIB_RESIDUAL_THRESH);
        return false;
    }

    // Accept new calibration
    memcpy(s_R, R_new, sizeof(s_R));
    s_residualRMS = residual;
    s_calibSamplesUsed = s_sampleCount;
    s_calibValid = true;

    ESP_LOGI(TAG, "Axis calibration OK: residual=%.3f, samples=%d", residual, s_sampleCount);
    ESP_LOGI(TAG, "R = [%.3f %.3f %.3f; %.3f %.3f %.3f; %.3f %.3f %.3f]",
             s_R[0], s_R[1], s_R[2], s_R[3], s_R[4], s_R[5], s_R[6], s_R[7], s_R[8]);

    return true;
}

bool axisCalibLoad(void)
{
    if (loadFromFile()) {
        ESP_LOGI(TAG, "Loaded calibration: residual=%.3f, samples=%d",
                 s_residualRMS, s_calibSamplesUsed);
        return true;
    }
    return false;
}

bool axisCalibSave(void)
{
    if (!s_calibValid) return false;
    if (saveToFile()) {
        ESP_LOGI(TAG, "Saved calibration to %s", CALIB_PATH);
        return true;
    }
    ESP_LOGW(TAG, "Failed to save calibration");
    return false;
}

bool axisCalibIsValid(void)
{
    return s_calibValid;
}

const float* axisCalibGetR(void)
{
    return s_R;
}

float axisCalibGetResidual(void)
{
    return s_residualRMS;
}

int axisCalibGetSampleCount(void)
{
    return s_calibSamplesUsed;
}
