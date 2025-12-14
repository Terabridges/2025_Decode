async function fetchList(){
  const res = await fetch('/api/list');
  if (!res.ok) return [];
  return await res.json();
}

function createCard(file){
  const el = document.createElement('div'); el.className='card';
  const title = document.createElement('h3'); title.textContent = file.name; el.appendChild(title);
  const meta = document.createElement('div'); meta.className='meta'; meta.textContent = `${file.size} bytes • ${file.modified}`; el.appendChild(meta);
  // Preview container (collapsed initially)
  const preview = document.createElement('pre'); preview.className = 'preview'; preview.style.display = 'none'; preview.textContent = '';
  el.appendChild(preview);

  const actions = document.createElement('div'); actions.className='actions';
  const previewBtn = document.createElement('button'); previewBtn.className='btn secondary'; previewBtn.textContent='Show preview';
  previewBtn.onclick = async () => {
    if (preview.style.display === 'none') {
      preview.style.display = 'block';
      preview.textContent = 'Loading...';
      try {
        const res = await fetch('/api/preview?file=' + encodeURIComponent(file.name));
        if (res.ok) {
          const txt = await res.text();
          preview.textContent = txt;
          previewBtn.textContent = 'Hide preview';
        } else {
          preview.textContent = 'Preview failed: ' + res.statusText;
        }
      } catch (e) { preview.textContent = 'Preview error'; }
    } else {
      preview.style.display = 'none'; previewBtn.textContent = 'Show preview';
    }
  };

  const dl = document.createElement('a'); dl.className='btn'; dl.href = file.url; dl.textContent='Download'; dl.setAttribute('download','');
  actions.appendChild(previewBtn);
  actions.appendChild(dl);
  el.appendChild(actions);
  return el;
}

async function refresh(){
  const grid = document.getElementById('grid'); grid.innerHTML='';
  const list = await fetchList();
  if (list.length===0){ grid.innerHTML='<div class="card"><div class="meta">No logs found</div></div>'; return; }
  list.forEach(f=> grid.appendChild(createCard(f)));
}

window.addEventListener('load', ()=>{ refresh(); setInterval(refresh, 30000); });
