const selectedFiles = new Map();
let latestResult = null;

const form = document.querySelector('#calibration-form');
const dropZone = document.querySelector('#drop-zone');
const imagePicker = document.querySelector('#image-picker');
const folderPicker = document.querySelector('#folder-picker');
const fileSummary = document.querySelector('#file-summary');
const runButton = document.querySelector('#run-button');
const statusPanel = document.querySelector('#status-panel');
const resultsPanel = document.querySelector('#results');

function addFiles(files) {
  [...files].filter(file => file.type.startsWith('image/') || /\.(jpe?g|png|bmp|tiff?)$/i.test(file.name))
    .forEach(file => selectedFiles.set(`${file.name}:${file.size}:${file.lastModified}`, file));
  const names = [...selectedFiles.values()].slice(0, 4).map(file => file.name);
  fileSummary.textContent = selectedFiles.size
    ? `${selectedFiles.size} image${selectedFiles.size === 1 ? '' : 's'} · ${names.join(', ')}${selectedFiles.size > 4 ? '…' : ''}`
    : 'No images selected';
  runButton.disabled = selectedFiles.size === 0;
}

[imagePicker, folderPicker].forEach(input => input.addEventListener('change', event => addFiles(event.target.files)));
['dragenter', 'dragover'].forEach(name => dropZone.addEventListener(name, event => {
  event.preventDefault(); dropZone.classList.add('dragging');
}));
['dragleave', 'drop'].forEach(name => dropZone.addEventListener(name, event => {
  event.preventDefault(); dropZone.classList.remove('dragging');
}));
dropZone.addEventListener('drop', event => addFiles(event.dataTransfer.files));

function fmt(value, digits = 3) {
  return Number.isFinite(value) ? Number(value).toFixed(digits) : '—';
}

function metric(label, value, detail, orange = false) {
  return `<div class="metric ${orange ? 'orange' : ''}"><div class="label">${label}</div><div class="value">${value}</div><div class="detail">${detail}</div></div>`;
}

function resultValue(result, key) {
  return result && result.success && Number.isFinite(result[key]) ? result[key] : null;
}

function renderParameterTable(target, clean, opencv, keys) {
  const rows = keys.map(key => `<tr><td>${key}</td><td>${fmt(clean?.[key], 4)}</td><td>${fmt(opencv?.[key], 4)}</td></tr>`).join('');
  target.innerHTML = `<table class="data-table"><thead><tr><th>Parameter</th><th>clean-calib</th><th>OpenCV</th></tr></thead><tbody>${rows}</tbody></table>`;
}

function renderBars(comparison) {
  const values = [comparison.clean_calib_rms_px, comparison.opencv_rms_px].filter(Number.isFinite);
  if (!values.length) return '<p>At least three shared detections are required.</p>';
  const max = Math.max(...values) * 1.15;
  return [
    ['clean-calib', comparison.clean_calib_rms_px, ''],
    ['OpenCV classic', comparison.opencv_rms_px, 'orange']
  ].map(([label, value, color]) => `<div class="comparison-row"><span>${label}</span><div class="comparison-track"><div class="comparison-fill ${color}" style="width:${100 * value / max}%"></div></div><span class="comparison-value">${fmt(value, 4)} px</span></div>`).join('');
}

function renderViewChart(cleanImages, opencvImages) {
  const cvByName = new Map(opencvImages.filter(x => x.found).map(x => [x.name, x]));
  const rows = cleanImages.filter(x => x.found && cvByName.has(x.name)).map(x => ({name: x.name, clean: x.rms_px, cv: cvByName.get(x.name).rms_px}));
  if (!rows.length) return '<p>No shared per-view measurements.</p>';
  const width = Math.max(780, rows.length * 76), height = 330, left = 48, top = 22, bottom = 62;
  const chartH = height - top - bottom, chartW = width - left - 20;
  const max = Math.max(.05, ...rows.flatMap(x => [x.clean, x.cv])) * 1.15;
  let svg = `<svg viewBox="0 0 ${width} ${height}" role="img" aria-label="Per-view RMS bar chart">`;
  for (let tick = 0; tick <= 4; tick++) {
    const value = max * tick / 4, y = top + chartH * (1 - tick / 4);
    svg += `<line x1="${left}" y1="${y}" x2="${width-20}" y2="${y}" stroke="#deded6"/><text x="${left-8}" y="${y+4}" text-anchor="end" font-size="10" fill="#68726b">${value.toFixed(2)}</text>`;
  }
  const group = chartW / rows.length, bar = Math.min(20, group * .28);
  rows.forEach((row, i) => {
    const center = left + group * (i + .5);
    [[row.clean, '#1f6b4f', -bar], [row.cv, '#e76f3c', 0]].forEach(([value, color, offset]) => {
      const h = chartH * value / max;
      svg += `<rect x="${center+offset}" y="${top+chartH-h}" width="${bar}" height="${h}" rx="2" fill="${color}"/>`;
    });
    svg += `<text x="${center}" y="${top+chartH+20}" text-anchor="middle" font-size="9" fill="#68726b">${row.name.replace(/\.[^.]+$/, '')}</text>`;
  });
  svg += `<circle cx="${width/2-80}" cy="${height-15}" r="5" fill="#1f6b4f"/><text x="${width/2-69}" y="${height-11}" font-size="10">clean-calib</text><circle cx="${width/2+25}" cy="${height-15}" r="5" fill="#e76f3c"/><text x="${width/2+36}" y="${height-11}" font-size="10">OpenCV</text></svg>`;
  return svg;
}

function renderImageTable(cleanImages, opencvImages) {
  const cleanByName = new Map(cleanImages.map(x => [x.name, x]));
  const cvByName = new Map(opencvImages.map(x => [x.name, x]));
  const names = [...new Set([...cleanByName.keys(), ...cvByName.keys()])].sort();
  const state = entry => entry ? `<span class="status-dot ${entry.found ? '' : 'fail'}"></span>${entry.found ? 'Found' : 'Miss'}` : '—';
  const body = names.map(name => {
    const clean = cleanByName.get(name), cv = cvByName.get(name);
    return `<tr><td>${name}</td><td>${state(clean)}</td><td>${fmt(clean?.rms_px, 4)}</td><td>${state(cv)}</td><td>${fmt(cv?.rms_px, 4)}</td><td>${clean?.error || cv?.error || ''}</td></tr>`;
  }).join('');
  return `<thead><tr><th>Image</th><th>Our detector</th><th>Our RMS</th><th>OpenCV</th><th>OpenCV RMS</th><th>Note</th></tr></thead><tbody>${body}</tbody>`;
}

function renderResults(data) {
  latestResult = data;
  const clean = data.clean_calib, cv = data.opencv, comparison = data.comparison;
  const cleanRms = resultValue(clean, 'rms_px'), cvRms = resultValue(cv, 'rms_px');
  document.querySelector('#metric-cards').innerHTML = [
    metric('Our detection', `${clean.detected || 0}/${data.dataset.images}`, `${fmt(clean.detection_rate_percent, 1)}% coverage`),
    metric('OpenCV detection', `${cv.detected || 0}/${data.dataset.images}`, `${fmt(cv.detection_rate_percent, 1)}% coverage`, true),
    metric('Our calibration RMS', cleanRms === null ? 'Failed' : `${fmt(cleanRms, 4)} px`, `${fmt(clean.total_time_ms, 0)} ms total`),
    metric('OpenCV calibration RMS', cvRms === null ? 'Failed' : `${fmt(cvRms, 4)} px`, `${fmt(cv.total_time_ms, 0)} ms total`, true)
  ].join('');
  document.querySelector('#rms-comparison').innerHTML = renderBars(comparison);
  renderParameterTable(document.querySelector('#intrinsics-table'), clean.intrinsics, cv.intrinsics, ['fx','fy','cx','cy','skew']);
  renderParameterTable(document.querySelector('#distortion-table'), clean.distortion, cv.distortion, ['k1','k2','k3','p1','p2']);
  document.querySelector('#view-chart').innerHTML = renderViewChart(clean.images || [], cv.images || []);
  document.querySelector('#image-table').innerHTML = renderImageTable(clean.images || [], cv.images || []);
  document.querySelector('#image-count').textContent = `${data.dataset.images} uploaded · ${comparison.shared_images || 0} shared`;
  resultsPanel.classList.remove('hidden');
  resultsPanel.scrollIntoView({behavior: 'smooth', block: 'start'});
}

form.addEventListener('submit', async event => {
  event.preventDefault();
  runButton.disabled = true;
  resultsPanel.classList.add('hidden');
  statusPanel.className = 'status-panel loading';
  statusPanel.textContent = `Analyzing ${selectedFiles.size} images with both pipelines…`;
  const payload = new FormData();
  selectedFiles.forEach(file => payload.append('images', file, file.name));
  payload.append('rows', document.querySelector('#rows').value);
  payload.append('cols', document.querySelector('#cols').value);
  payload.append('square_size', document.querySelector('#square-size').value);
  try {
    const response = await fetch('/api/calibrate', {method: 'POST', body: payload});
    const data = await response.json();
    if (!response.ok) throw new Error(data.error || 'Calibration request failed');
    statusPanel.classList.add('hidden');
    renderResults(data);
  } catch (error) {
    statusPanel.className = 'status-panel error';
    statusPanel.textContent = error.message;
  } finally {
    runButton.disabled = selectedFiles.size === 0;
  }
});

document.querySelector('#download-json').addEventListener('click', () => {
  if (!latestResult) return;
  const blob = new Blob([JSON.stringify(latestResult, null, 2)], {type: 'application/json'});
  const link = document.createElement('a');
  link.href = URL.createObjectURL(blob);
  link.download = 'clean-calib-report.json';
  link.click();
  URL.revokeObjectURL(link.href);
});

fetch('/api/health').then(response => response.json()).then(data => {
  const health = document.querySelector('#health');
  health.textContent = data.opencv.available ? `● Ready · OpenCV ${data.opencv.version}` : '● Ready · OpenCV unavailable';
  health.classList.add('ready');
}).catch(() => { document.querySelector('#health').textContent = 'Runtime check failed'; });
