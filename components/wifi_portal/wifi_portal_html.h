/**
 * @file wifi_portal_html.h
 * @brief Embedded HTML pages for WiFi captive portal
 *
 * 로그 뷰어 + 설정 페이지 (C string literals)
 */

#ifndef WIFI_PORTAL_HTML_H
#define WIFI_PORTAL_HTML_H

// ============================================================
// 로그 뷰어 페이지 (메인 페이지)
// ============================================================
static const char HTML_LOG_VIEWER[] = R"rawhtml(<!DOCTYPE html>
<html><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>GPS Laptimer - Log</title>
<link rel="icon" href="/favicon.ico" type="image/svg+xml">
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{background:#1a1a2e;color:#e0e0e0;font-family:'Courier New',monospace;font-size:13px}
#header{background:#16213e;padding:8px 12px;display:flex;justify-content:space-between;align-items:center;border-bottom:1px solid #0f3460}
#header h1{font-size:16px;color:#e94560}
#header a{color:#53a8b6;text-decoration:none;font-size:13px}
#status{padding:4px 12px;font-size:11px;color:#888}
#log{padding:8px 12px;overflow-y:auto;white-space:pre-wrap;word-break:break-all;height:calc(100vh - 110px)}
.btn{background:#0f3460;color:#e0e0e0;border:1px solid #53a8b6;padding:4px 10px;cursor:pointer;font-size:12px;border-radius:3px}
.btn:hover{background:#1a4a7a}
.btn.active{background:#e94560;border-color:#e94560;color:white}
#controls{padding:4px 12px;display:flex;gap:6px;flex-wrap:wrap;align-items:center}
#filters{padding:2px 12px;display:flex;gap:4px;flex-wrap:wrap;align-items:center}
#filters label{color:#888;font-size:11px;margin-right:4px}
#customFilter{background:#0f3460;color:#e0e0e0;border:1px solid #53a8b6;padding:3px 8px;font-size:12px;border-radius:3px;width:120px}
.log-E{color:#ff6b6b}.log-W{color:#ffd93d}.log-I{color:#6bcb77}.log-D{color:#a0a0a0}
.cnt{font-size:10px;color:#888;margin-left:2px}
</style>
</head><body>
<div id="header">
  <h1>GPS Laptimer Log</h1>
  <div><a href="/ota">Update</a> &middot; <a href="/settings">Settings</a></div>
</div>
<div id="controls">
  <button class="btn" onclick="clearLog()">Clear</button>
  <button class="btn" id="scrollBtn" onclick="toggleScroll()">Scroll: ON</button>
</div>
<div id="filters">
  <label>Filter:</label>
  <button class="btn active" onclick="setFilter('')" id="f_all">ALL</button>
  <button class="btn" onclick="setFilter('BAT')" id="f_BAT">BAT</button>
  <button class="btn" onclick="setFilter('LAP')" id="f_LAP">LAP</button>
  <button class="btn" onclick="setFilter('GPS')" id="f_GPS">GPS</button>
  <button class="btn" onclick="setFilter('DISPLAY')" id="f_DISPLAY">DISP</button>
  <button class="btn" onclick="setFilter('wifi')" id="f_wifi">WIFI</button>
  <input type="text" id="customFilter" placeholder="custom..." oninput="setCustomFilter(this.value)">
  <span id="lineCount" class="cnt"></span>
</div>
<div id="status">Connecting...</div>
<div id="log"></div>
<script>
var logEl=document.getElementById('log'),statusEl=document.getElementById('status');
var ws,autoScroll=true,reconnectTimer;
var allLines=[],activeFilter='',maxLines=3000;

var FILTERS={
  'BAT':  /BAT |battery|batteryV/i,
  'LAP':  /LAP |lap |SECTOR|sector|FINISH|onLapComplete/i,
  'GPS':  /GPS |gps |sats|ublox|NMEA|NAV-PVT/i,
  'DISPLAY': /DISPLAY|display|LVGL|lvgl/i,
  'wifi': /wifi_portal|dns_server|httpd|WS client|SoftAP/i
};

function matchFilter(line){
  if(!activeFilter) return true;
  var re=FILTERS[activeFilter];
  if(re) return re.test(line);
  return line.toLowerCase().indexOf(activeFilter.toLowerCase())>=0;
}
function colorize(line){
  if(line.indexOf('E (')===0) return '<span class="log-E">'+line+'</span>';
  if(line.indexOf('W (')===0) return '<span class="log-W">'+line+'</span>';
  if(line.indexOf('I (')===0) return '<span class="log-I">'+line+'</span>';
  if(line.indexOf('D (')===0) return '<span class="log-D">'+line+'</span>';
  return line;
}
function renderVisible(){
  var html='',shown=0;
  for(var i=0;i<allLines.length;i++){
    if(matchFilter(allLines[i])){html+=colorize(allLines[i])+'\n';shown++}
  }
  logEl.innerHTML=html;
  document.getElementById('lineCount').textContent=shown+'/'+allLines.length;
  if(autoScroll) logEl.scrollTop=logEl.scrollHeight;
}
function addLines(text){
  var lines=text.split('\n');
  for(var i=0;i<lines.length;i++){
    if(lines[i].length>0) allLines.push(lines[i]);
  }
  if(allLines.length>maxLines) allLines=allLines.slice(allLines.length-maxLines);
  // 증분 렌더링 (ALL 필터이거나 새 줄이 필터 매치할 때)
  var newHtml='';
  for(var i=allLines.length-lines.length;i<allLines.length;i++){
    if(i>=0 && matchFilter(allLines[i])){newHtml+=colorize(allLines[i])+'\n'}
  }
  if(newHtml){
    logEl.innerHTML+=newHtml;
    var cnt=document.getElementById('lineCount');
    var shown=parseInt(cnt.textContent)||0;
    cnt.textContent=(shown+newHtml.split('\n').length-1)+'/'+allLines.length;
  }
  if(autoScroll) logEl.scrollTop=logEl.scrollHeight;
}
function setFilter(f){
  activeFilter=f;
  document.getElementById('customFilter').value=f && !FILTERS[f]?f:'';
  var btns=document.getElementById('filters').querySelectorAll('.btn');
  for(var i=0;i<btns.length;i++) btns[i].classList.remove('active');
  var id=f?'f_'+f:'f_all';
  var el=document.getElementById(id);
  if(el) el.classList.add('active');
  renderVisible();
}
function setCustomFilter(v){
  activeFilter=v;
  var btns=document.getElementById('filters').querySelectorAll('.btn');
  for(var i=0;i<btns.length;i++) btns[i].classList.remove('active');
  if(!v) document.getElementById('f_all').classList.add('active');
  renderVisible();
}
function connect(){
  var host=location.host||'192.168.4.1';
  ws=new WebSocket('ws://'+host+'/ws/log');
  ws.onopen=function(){statusEl.textContent='Connected';clearTimeout(reconnectTimer)};
  ws.onclose=function(){statusEl.textContent='Disconnected - reconnecting...';reconnectTimer=setTimeout(connect,2000)};
  ws.onerror=function(){ws.close()};
  ws.onmessage=function(e){addLines(e.data)};
}
function clearLog(){allLines=[];logEl.innerHTML='';document.getElementById('lineCount').textContent='0/0'}
function toggleScroll(){
  autoScroll=!autoScroll;
  document.getElementById('scrollBtn').textContent='Scroll: '+(autoScroll?'ON':'OFF');
}
connect();
</script>
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
<a class="back" href="/">&larr; Log</a> &middot; <a class="back" href="/ota">Firmware Update</a>
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
<a class="back" href="/">&larr; Back to Log</a>
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
