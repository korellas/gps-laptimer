/**
 * @file wifi_portal_html.h
 * @brief Embedded HTML pages for WiFi portal
 *
 * 로그 뷰어 + 설정 페이지 (C string literals)
 */

#ifndef WIFI_PORTAL_HTML_H
#define WIFI_PORTAL_HTML_H

// ============================================================
// 메인 페이지 (경량 네비게이션 허브 — JS/WS 없음)
// ============================================================
static const char HTML_LOG_VIEWER[] = R"rawhtml(<!DOCTYPE html>
<html><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>GPS Laptimer</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{background:#1a1a2e;color:#e0e0e0;font-family:sans-serif;padding:24px;max-width:400px;margin:0 auto}
h1{color:#e94560;font-size:22px;margin-bottom:20px}
a.card{display:block;background:#16213e;border:1px solid #0f3460;border-radius:8px;padding:16px;margin-bottom:12px;text-decoration:none;color:#e0e0e0}
a.card:hover{border-color:#e94560}
a.card h2{color:#4af;font-size:16px;margin-bottom:4px}
a.card p{color:#888;font-size:13px}
.info{color:#666;font-size:11px;margin-top:20px;text-align:center}
</style>
</head><body>
<h1>GPS Laptimer</h1>
<a class="card" href="/files"><h2>SD Card Files</h2><p>Browse logs and lap data</p></a>
<a class="card" href="/settings"><h2>Settings</h2><p>Phone number, display options</p></a>
<a class="card" href="/ota"><h2>Firmware Update</h2><p>Upload new firmware (.bin)</p></a>
<div class="info">192.168.4.1</div>
</body></html>)rawhtml";

// ============================================================
// 설정 페이지
// ============================================================
static const char HTML_SETTINGS[] = R"rawhtml(<!DOCTYPE html>
<html><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>GPS Laptimer - Settings</title>
<link rel="icon" href="/favicon.ico" type="image/svg+xml">
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{background:#1a1a2e;color:#e0e0e0;font-family:sans-serif;font-size:14px;padding:16px}
h1{color:#e94560;font-size:20px;margin-bottom:16px}
a{color:#53a8b6}
.card{background:#16213e;border:1px solid #0f3460;border-radius:6px;padding:16px;margin-bottom:12px}
label{display:block;margin-bottom:4px;color:#aaa;font-size:12px}
input[type=text]{width:100%%;background:#0f3460;color:#e0e0e0;border:1px solid #53a8b6;padding:8px;font-size:16px;border-radius:3px;margin-bottom:12px}
.btn{background:#e94560;color:white;border:none;padding:10px 24px;cursor:pointer;font-size:14px;border-radius:4px}
.btn:hover{background:#c73852}
#msg{color:#6bcb77;margin-top:8px;display:none}
.back{margin-bottom:12px;display:inline-block}
</style>
</head><body>
<a class="back" href="/">&larr; Home</a> &middot; <a class="back" href="/files">Files</a> &middot; <a class="back" href="/ota">Firmware Update</a>
<h1>Settings</h1>
<form id="form" class="card">
  <label for="phone">Phone Number (displayed on screen)</label>
  <input type="text" id="phone" name="phone" placeholder="010-1234-5678" maxlength="31">
  <button type="submit" class="btn">Save</button>
  <div id="msg">Saved!</div>
</form>
<script>
// 현재 설정 로드
fetch('/api/settings').then(r=>r.json()).then(function(d){
  if(d.phone)document.getElementById('phone').value=d.phone;
});
document.getElementById('form').onsubmit=function(e){
  e.preventDefault();
  var data=JSON.stringify({phone:document.getElementById('phone').value});
  fetch('/api/settings',{method:'POST',headers:{'Content-Type':'application/json'},body:data})
  .then(function(r){
    if(r.ok){var m=document.getElementById('msg');m.style.display='block';setTimeout(function(){m.style.display='none'},2000)}
  });
};
</script>
</body></html>)rawhtml";

// ============================================================
// OTA 업데이트 페이지
// ============================================================
static const char HTML_OTA[] = R"rawhtml(<!DOCTYPE html>
<html><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>GPS Laptimer - Firmware Update</title>
<link rel="icon" href="/favicon.ico" type="image/svg+xml">
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{background:#1a1a2e;color:#e0e0e0;font-family:sans-serif;font-size:14px;padding:16px}
h1{color:#e94560;font-size:20px;margin-bottom:16px}
a{color:#53a8b6}
.card{background:#16213e;border:1px solid #0f3460;border-radius:6px;padding:16px;margin-bottom:12px}
.ver{font-size:18px;color:#6bcb77;font-family:'Courier New',monospace}
label{display:block;margin-bottom:8px;color:#aaa;font-size:12px}
.btn{background:#e94560;color:white;border:none;padding:10px 24px;cursor:pointer;font-size:14px;border-radius:4px;width:100%}
.btn:hover{background:#c73852}
.btn:disabled{background:#555;cursor:not-allowed}
.progress-bar{background:#0f3460;border-radius:4px;height:24px;overflow:hidden;margin:8px 0}
.progress-fill{background:#e94560;height:100%;width:0%;transition:width 0.3s;border-radius:4px}
.pct{text-align:center;font-size:13px;color:#aaa}
.warn{color:#ffd93d;font-size:12px;margin-top:8px}
.ok{color:#6bcb77;font-size:16px}
.err{color:#ff6b6b;font-size:16px}
.fi{color:#888;font-size:12px;margin-bottom:8px}
.back{margin-bottom:12px;display:inline-block}
.hid{display:none}
</style>
</head><body>
<a class="back" href="/">&larr; Home</a> &middot; <a class="back" href="/files">Files</a>
<h1>Firmware Update</h1>

<div class="card">
  <label>Current Firmware</label>
  <div class="ver" id="ver">loading...</div>
</div>

<div class="card" id="upCard">
  <label>Select firmware file (.bin)</label>
  <input type="file" id="file" accept=".bin" onchange="onFile()">
  <div class="fi" id="fi"></div>
  <button class="btn" id="ubtn" onclick="doUpload()" disabled>Upload &amp; Update</button>
  <div class="warn">Do not disconnect power during update!</div>
</div>

<div class="card hid" id="progCard">
  <div id="stMsg">Uploading firmware...</div>
  <div class="progress-bar"><div class="progress-fill" id="pFill"></div></div>
  <div class="pct" id="pTxt">0%</div>
</div>

<div class="card hid" id="resCard">
  <div id="resMsg"></div>
</div>

<script>
fetch('/api/ota/status').then(function(r){return r.json()}).then(function(d){
  document.getElementById('ver').textContent=d.version||'unknown';
}).catch(function(){document.getElementById('ver').textContent='error'});

function onFile(){
  var f=document.getElementById('file').files[0];
  var b=document.getElementById('ubtn');
  var i=document.getElementById('fi');
  if(f){
    i.textContent=f.name+' ('+(f.size/1024/1024).toFixed(2)+' MB)';
    b.disabled=false;
  }else{i.textContent='';b.disabled=true}
}

function doUpload(){
  var f=document.getElementById('file').files[0];
  if(!f)return;
  document.getElementById('upCard').style.display='none';
  document.getElementById('progCard').classList.remove('hid');
  var xhr=new XMLHttpRequest();
  xhr.open('POST','/api/ota/upload');
  xhr.setRequestHeader('Content-Type','application/octet-stream');
  xhr.upload.onprogress=function(e){
    if(e.lengthComputable){
      var p=Math.round(e.loaded/e.total*100);
      document.getElementById('pFill').style.width=p+'%';
      document.getElementById('pTxt').textContent=p+'%';
      document.getElementById('stMsg').textContent=p<100?'Uploading firmware...':'Validating...';
    }
  };
  xhr.onload=function(){
    try{
      var r=JSON.parse(xhr.responseText);
      if(r.ok){
        document.getElementById('progCard').classList.add('hid');
        document.getElementById('resCard').classList.remove('hid');
        document.getElementById('resMsg').innerHTML=
          '<div class="ok">Update successful!</div>'+
          '<div style="margin-top:8px;color:#aaa">Device is rebooting.<br>Reconnect to LAPTIMER WiFi after restart.</div>';
      }else{fail(r.error||'Update failed')}
    }catch(e){fail('Invalid response')}
  };
  xhr.onerror=function(){fail('Connection lost')};
  xhr.ontimeout=function(){fail('Upload timed out')};
  xhr.timeout=300000;
  xhr.send(f);
}

function fail(m){
  document.getElementById('progCard').classList.add('hid');
  document.getElementById('resCard').classList.remove('hid');
  document.getElementById('resMsg').innerHTML=
    '<div class="err">'+m+'</div>'+
    '<div style="margin-top:8px"><a href="/ota">Try again</a></div>';
}
</script>
</body></html>)rawhtml";

#endif // WIFI_PORTAL_HTML_H
