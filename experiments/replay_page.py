"""Canlandirma sayfasini (HTML) replay_2026.json'dan uretir."""
import json
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA = os.path.join(ROOT, "results", "replay", "replay_2026.json")
OUT = os.path.join(ROOT, "results", "replay", "replay_2026.html")

TEMPLATE = r"""<!doctype html>
<html lang="tr"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>2026 yörünge canlandırması</title>
<style>
:root{--bg:#fbfbf9;--fg:#1d1d1b;--mut:#6b6b66;--gt:#111;--orb:#2b6cb0;--sp:#c53030;--line:#e4e4df}
@media (prefers-color-scheme: dark){:root:not([data-theme="light"]){--bg:#161615;--fg:#eeeeea;--mut:#9a9a94;--gt:#f2f2ee;--orb:#6fa8e6;--sp:#f08080;--line:#333330}}
:root[data-theme="dark"]{--bg:#161615;--fg:#eeeeea;--mut:#9a9a94;--gt:#f2f2ee;--orb:#6fa8e6;--sp:#f08080;--line:#333330}
body{margin:0;background:var(--bg);color:var(--fg);font:14px/1.5 system-ui,-apple-system,"Segoe UI",sans-serif;padding:16px}
h1{font-size:18px;margin:0 0 4px}
.note{color:var(--mut);font-size:12.5px;margin:0 0 12px;max-width:820px}
canvas{width:100%;max-width:820px;height:auto;aspect-ratio:820/560;border:1px solid var(--line);border-radius:8px;display:block;background:var(--bg)}
.row{display:flex;flex-wrap:wrap;gap:10px;align-items:center;margin:12px 0;max-width:820px}
button,select{font:inherit;color:var(--fg);background:transparent;border:1px solid var(--line);border-radius:6px;padding:6px 10px;cursor:pointer}
input[type=range]{flex:1;min-width:180px}
.legend{display:flex;gap:14px;font-size:12.5px;color:var(--mut)}
.sw{display:inline-block;width:14px;height:3px;vertical-align:middle;margin-right:5px}
.stats{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:8px;max-width:820px;margin-top:6px}
.stat{border:1px solid var(--line);border-radius:8px;padding:8px 10px}
.stat b{display:block;font-size:12px;color:var(--mut);font-weight:500}
.stat span{font-size:18px}
footer{color:var(--mut);font-size:12px;max-width:820px;margin-top:14px}
</style></head>
<body>
<h1>2026 uçuşu: tahmin yörüngesinin GT ile karşılaştırılması</h1>
<p class="note">Bu, <b>kayıtlı</b> adım çıktısının canlandırmasıdır; sistem bu sayfada canlı çalışmaz. Şekil görünümü GT'ye tüm uçuş boyunca hizalıdır (ideal ölçüm). Üretim görünümü yalnızca ilk 450 karedeki GT ile hizalanır ve sonrasında GT kullanılmaz.</p>
<canvas id="c" width="820" height="560"></canvas>
<div class="row">
  <button id="play">▶ Oynat</button>
  <select id="speed"><option value="1">1×</option><option value="2" selected>2×</option><option value="4">4×</option><option value="8">8×</option></select>
  <select id="view"><option value="shape">Şekil (GT hizalı)</option><option value="prod">Üretim çıktısı</option></select>
  <input id="scrub" type="range" min="0" max="0" value="0">
</div>
<div class="legend">
  <span><i class="sw" style="background:var(--gt)"></i>GT</span>
  <span><i class="sw" style="background:var(--orb)"></i>ORB</span>
  <span><i class="sw" style="background:var(--sp)"></i>SuperPoint+LG</span>
</div>
<div class="stats">
  <div class="stat"><b>kare</b><span id="st-frame">–</span></div>
  <div class="stat"><b>ORB anlık hata (m)</b><span id="st-orb">–</span></div>
  <div class="stat"><b>SuperPoint anlık hata (m)</b><span id="st-sp">–</span></div>
</div>
<footer>Hata değerleri, seçilen görünüme göre anlık konum farkıdır. Metinde verilen şekil hatası, tüm uçuşun ortalamasıdır; anlık değerler bundan farklı olabilir. Veri ve GT kullanım izni kapsamında gösterilmektedir.</footer>
<script>
const D = __DATA__;
const cv = document.getElementById('c'), ctx = cv.getContext('2d');
const css = getComputedStyle(document.documentElement);
const col = {gt: css.getPropertyValue('--gt').trim() || '#111', orb: css.getPropertyValue('--orb').trim() || '#2b6cb0', sp: css.getPropertyValue('--sp').trim() || '#c53030', line: css.getPropertyValue('--line').trim() || '#ddd', mut: css.getPropertyValue('--mut').trim() || '#666'};
const N = D.frames.length;
const scrub = document.getElementById('scrub');
scrub.max = N - 1;
let i = 0, playing = false, last = 0, speed = 2, view = 'shape';

function allPts(){
  const pts = [...D.gt];
  for (const k in D.series) { pts.push(...D.series[k].shape, ...D.series[k].prod); }
  return pts;
}
const P = allPts();
let minx = Math.min(...P.map(p=>p[0])), maxx = Math.max(...P.map(p=>p[0]));
let miny = Math.min(...P.map(p=>p[1])), maxy = Math.max(...P.map(p=>p[1]));
const pad = 20;
function tf(p){
  const W = cv.width - 2*pad, H = cv.height - 2*pad;
  const s = Math.min(W/(maxx-minx), H/(maxy-miny));
  const ox = pad + (W - s*(maxx-minx))/2, oy = pad + (H - s*(maxy-miny))/2;
  return [ox + (p[0]-minx)*s, cv.height - (oy + (p[1]-miny)*s)];
}
function path(pts, upto, color, width){
  ctx.beginPath();
  for (let k=0; k<=upto && k<pts.length; k++){
    const [x,y] = tf(pts[k]);
    k===0 ? ctx.moveTo(x,y) : ctx.lineTo(x,y);
  }
  ctx.strokeStyle = color; ctx.lineWidth = width; ctx.stroke();
}
function dot(p, color){
  const [x,y] = tf(p);
  ctx.beginPath(); ctx.arc(x,y,5,0,Math.PI*2); ctx.fillStyle = color; ctx.fill();
}
function draw(){
  ctx.clearRect(0,0,cv.width,cv.height);
  ctx.strokeStyle = col.line; ctx.lineWidth = 1;
  ctx.strokeRect(pad/2, pad/2, cv.width-pad, cv.height-pad);
  path(D.gt, i, col.gt, 2.5);
  const o = D.series['ORB'], s = D.series['SuperPoint+LG'];
  const oP = view==='shape' ? o.shape : o.prod, sP = view==='shape' ? s.shape : s.prod;
  path(oP, i, col.orb, 2);
  path(sP, i, col.sp, 2);
  dot(D.gt[i], col.gt); dot(oP[i], col.orb); dot(sP[i], col.sp);
  const eo = view==='shape' ? o.err_shape : o.err_prod, es = view==='shape' ? s.err_shape : s.err_prod;
  document.getElementById('st-frame').textContent = D.frames[i] + ' / ' + D.frames[N-1];
  document.getElementById('st-orb').textContent = eo[i].toFixed(1);
  document.getElementById('st-sp').textContent = es[i].toFixed(1);
  scrub.value = i;
}
function loop(ts){
  if (playing){
    if (!last) last = ts;
    const dt = ts - last; last = ts;
    const step = Math.max(1, Math.round(speed * dt / 60));
    i = Math.min(N-1, i + step);
    if (i >= N-1) { playing = false; document.getElementById('play').textContent = '▶ Oynat'; }
    draw();
  } else { last = 0; }
  requestAnimationFrame(loop);
}
document.getElementById('play').onclick = () => {
  if (i >= N-1) i = 0;
  playing = !playing;
  document.getElementById('play').textContent = playing ? '❚❚ Durdur' : '▶ Oynat';
};
document.getElementById('speed').onchange = e => speed = +e.target.value;
document.getElementById('view').onchange = e => { view = e.target.value; draw(); };
scrub.oninput = e => { i = +e.target.value; draw(); };
draw(); requestAnimationFrame(loop);
</script></body></html>
"""

def main():
    with open(DATA, encoding="utf-8") as fh:
        data = json.load(fh)
    html = TEMPLATE.replace("__DATA__", json.dumps(data, ensure_ascii=False))
    with open(OUT, "w", encoding="utf-8") as fh:
        fh.write(html)
    print("yazildi:", OUT)

if __name__ == "__main__":
    main()
