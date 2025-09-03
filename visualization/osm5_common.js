// Hardened version with robust path coercion and logging

async function loadData(jsonPath) {
  const resp = await fetch(jsonPath);
  if (!resp.ok) throw new Error("Failed to load JSON: " + jsonPath + " status=" + resp.status);
  const data = await resp.json();
  console.log("Loaded JSON:", data);
  return data;
}

function buildNodeIndex(nodes) {
  const idx = {};
  nodes.forEach(n => { idx[n.id] = n; });
  return idx;
}

/**
 * Attempt to coerce an incoming value into an array of integers.
 * Accepts:
 *  - Array<number>
 *  - String containing JSON array or comma/space separated numbers
 *  - Single number
 *  - Object of shape { path: [...] }
 * Returns [] if cannot parse.
 */
function coercePath(val) {
  if (Array.isArray(val)) return val;
  if (val == null) return [];
  if (typeof val === 'number') return [val];
  if (typeof val === 'object') {
    // Maybe nested { path: [...] } or similar
    if (Array.isArray(val.path)) return val.path;
    // Collect numeric own properties like {"0":123,"1":456}
    const numericKeys = Object.keys(val).filter(k => /^\d+$/.test(k));
    if (numericKeys.length) {
      return numericKeys
        .sort((a,b)=>+a - +b)
        .map(k => val[k])
        .filter(x => Number.isFinite(x));
    }
    return [];
  }
  if (typeof val === 'string') {
    const trimmed = val.trim();
    // Try JSON parse first
    if ((trimmed.startsWith('[') && trimmed.endsWith(']')) ||
        (trimmed.startsWith('"[') && trimmed.endsWith(']"'))) {
      try {
        const parsed = JSON.parse(trimmed.startsWith('"[') ? trimmed.slice(1,-1) : trimmed);
        if (Array.isArray(parsed)) return parsed;
      } catch (e) {
        console.warn("Failed JSON parse of path string:", e);
      }
    }
    // Fallback: split on non-digits
    const nums = trimmed.split(/[^0-9]+/).filter(Boolean).map(x => +x);
    if (nums.length) return nums;
  }
  return [];
}

function pathToLatLngs(pathIds, nodeIndex) {
  const coerced = coercePath(pathIds);
  if (!Array.isArray(coerced)) {
    console.warn("Could not coerce path to array:", pathIds);
    return [];
  }
  return coerced
    .filter(id => nodeIndex[id] !== undefined)
    .map(id => [nodeIndex[id].lat, nodeIndex[id].lon]);
}

function addBaseLayers(map, data) {
  const nodeIndex = buildNodeIndex(data.nodes);

  const baseLayer = L.layerGroup();
  const shortcutLayer = L.layerGroup();
  
  // Draw regular edges
  (data.regular_edges || []).forEach(e => {
    const u = nodeIndex[e.u], v = nodeIndex[e.v];
    if (!u || !v) return;
    L.polyline([[u.lat,u.lon],[v.lat,v.lon]], {
      color: '#888', weight: 1, opacity: 0.5
    }).addTo(baseLayer);
  });

  // Draw shortcut edges
  (data.shortcut_edges || []).forEach(e => {
    const u = nodeIndex[e.u], v = nodeIndex[e.v];
    if (!u || !v) return;
    L.polyline([[u.lat,u.lon],[v.lat,v.lon]], {
      color: 'purple', weight: 2, opacity: 0.7, dashArray: '3, 3'
    }).addTo(shortcutLayer);
  });

  baseLayer.addTo(map);
  return { baseLayer, shortcutLayer, nodeIndex };
}

function addExamplePaths(map, data, exampleIndex, nodeIndex) {
  if (!data.examples || !Array.isArray(data.examples) || data.examples.length === 0) {
    console.warn("No examples in data.");
    return null;
  }
  if (exampleIndex < 0 || exampleIndex >= data.examples.length) {
    console.warn("Invalid example index", exampleIndex, "using 0");
    exampleIndex = 0;
  }
  const ex = data.examples[exampleIndex];
  console.log("Using example index", exampleIndex, ex);

  const dijkstraLatLngs = pathToLatLngs(ex.dijkstra_path, nodeIndex);
  const chShortcutLatLngs = pathToLatLngs(ex.ch_path_shortcut, nodeIndex);
  const chUnpackedLatLngs = pathToLatLngs(ex.ch_path_unpacked, nodeIndex);

  // If any path is single point, draw a marker instead so user sees something.
  function addPathOrMarker(latlngs, opts, label) {
    if (latlngs.length === 0) {
      console.warn(label, "path empty -> skipping");
      return null;
    }
    if (latlngs.length === 1) {
      return L.circleMarker(latlngs[0], {
        radius: 6, color: opts.color || 'black', fillColor: opts.color || 'black', fillOpacity: 0.9
      }).bindTooltip(label + " (single node)").addTo(map);
    }
    return L.polyline(latlngs, opts).addTo(map);
  }

  const dijkstraLayer = addPathOrMarker(dijkstraLatLngs, {
    color: 'blue', weight: 5, opacity: 0.75
  }, "Dijkstra");
  const chShortcutLayer = addPathOrMarker(chShortcutLatLngs, {
    color: 'red', weight: 4, dashArray: '6,4', opacity: 0.75
  }, "CH (shortcut)");
  const chUnpackedLayer = addPathOrMarker(chUnpackedLatLngs, {
    color: 'lime', weight: 4, opacity: 0.75
  }, "CH (unpacked)");

  const allLayers = [dijkstraLayer, chShortcutLayer, chUnpackedLayer].filter(Boolean);

  if (allLayers.length) {
    const group = L.featureGroup(allLayers);
    const bounds = group.getBounds();
    if (bounds.isValid()) map.fitBounds(bounds, { padding: [20,20] });
  } else {
    console.warn("No valid path layers to fit.");
  }

  return {
    dijkstraLine: dijkstraLayer,
    chShortcutLine: chShortcutLayer,
    chUnpackedLine: chUnpackedLayer
  };
}

function addLegend(map) {
  const legend = L.control({position: 'bottomright'});
  legend.onAdd = function() {
    const div = L.DomUtil.create('div','info legend');
    div.innerHTML = `
      <style>
        .legend { background: rgba(255,255,255,0.85); padding:8px; font:12px sans-serif; line-height:1.3; }
        .legend-item { margin-bottom:4px; white-space:nowrap; }
        .swatch { display:inline-block; width:16px; height:6px; margin-right:6px; vertical-align:middle; border-radius:2px; }
      </style>
      <div class="legend">
        <div class="legend-item"><span class="swatch" style="background:#888;"></span>Original Graph</div>
        <div class="legend-item"><span class="swatch" style="background:purple;"></span>Shortcut Edge</div>
        <div class="legend-item"><span class="swatch" style="background:blue;"></span>Dijkstra Path</div>
        <div class="legend-item"><span class="swatch" style="background:red;"></span>CH Path (hierarchy)</div>
        <div class="legend-item"><span class="swatch" style="background:lime;"></span>Unpacked CH Path</div>
      </div>
    `;
    return div;
  };
  legend.addTo(map);
}

async function initExample(exampleIndex) {
  try {
    const data = await loadData('../results/part_1/osm5_visualization.json');

    let avgLat = 0, avgLon = 0;
    data.nodes.forEach(n => { avgLat += n.lat; avgLon += n.lon; });
    if (data.nodes.length) { avgLat /= data.nodes.length; avgLon /= data.nodes.length; }

    const map = L.map('map').setView([avgLat, avgLon], 12);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
       maxZoom: 19,
       attribution: '&copy; OpenStreetMap contributors'
    }).addTo(map);

    const { baseLayer, shortcutLayer, nodeIndex } = addBaseLayers(map, data);
    const pathLayers = addExamplePaths(map, data, exampleIndex, nodeIndex);
    addLegend(map);

    const overlays = {
      "Original Graph": baseLayer,
      "Shortcut Edges": shortcutLayer
    };
    if (pathLayers) {
      if (pathLayers.dijkstraLine) overlays["Dijkstra Path"] = pathLayers.dijkstraLine;
      if (pathLayers.chShortcutLine) overlays["CH Path (shortcut)"] = pathLayers.chShortcutLine;
      if (pathLayers.chUnpackedLine) overlays["CH Path (unpacked)"] = pathLayers.chUnpackedLine;
    } else {
      console.warn("No pathLayers returned.");
    }
    L.control.layers({}, overlays, { collapsed: false }).addTo(map);
  } catch (e) {
    console.error("initExample failed:", e);
  }
}

async function initCHGraphOnly() {
  try {
    const data = await loadData('../results/part_1/osm5_visualization.json');
    let avgLat = 0, avgLon = 0;
    data.nodes.forEach(n => { avgLat += n.lat; avgLon += n.lon; });
    if (data.nodes.length) { avgLat /= data.nodes.length; avgLon /= data.nodes.length; }

    const map = L.map('map').setView([avgLat, avgLon], 12);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
       maxZoom: 19,
       attribution: '&copy; OpenStreetMap contributors'
    }).addTo(map);

    const nodeIndex = buildNodeIndex(data.nodes);
    
    // Create separate layers for regular edges and shortcuts
    const regularEdgeLayer = L.layerGroup();
    const shortcutLayer = L.layerGroup();
    
    // Arrays to collect all polylines for bounds calculation
    const allPolylines = [];
    
    // Draw regular edges
    (data.regular_edges || []).forEach(e => {
      const u = nodeIndex[e.u], v = nodeIndex[e.v];
      if (!u || !v) return;
      const polyline = L.polyline([[u.lat,u.lon],[v.lat,v.lon]], {
        color: '#3388ff',
        weight: 1.5,
        opacity: 0.6
      });
      polyline.addTo(regularEdgeLayer);
      allPolylines.push(polyline);
    });
    
    // Draw shortcut edges
    (data.shortcut_edges || []).forEach(e => {
      const u = nodeIndex[e.u], v = nodeIndex[e.v];
      if (!u || !v) return;
      const polyline = L.polyline([[u.lat,u.lon],[v.lat,v.lon]], {
        color: 'purple',
        weight: 2,
        opacity: 0.7,
        dashArray: '5, 5'
      });
      polyline.addTo(shortcutLayer);
      allPolylines.push(polyline);
    });
    
    // Add layers to map
    regularEdgeLayer.addTo(map);
    shortcutLayer.addTo(map);
    
    // Calculate bounds using the collected polylines
    if (allPolylines.length > 0) {
      const group = L.featureGroup(allPolylines);
      const bounds = group.getBounds();
      if (bounds.isValid()) {
        map.fitBounds(bounds, {padding:[20,20]});
      }
    } else {
      // Fallback to center view if no edges
      map.setView([avgLat, avgLon], 12);
    }
    
    // Use Leaflet's built-in layer control
    const overlays = {
      "Regular Edges": regularEdgeLayer,
      "Shortcut Edges": shortcutLayer
    };
    L.control.layers(null, overlays, { collapsed: false }).addTo(map);
    
    // Update the legend
    const legend = L.control({position: 'bottomright'});
    legend.onAdd = function() {
      const div = L.DomUtil.create('div','info legend');
      div.innerHTML = `
        <style>
          .legend { background: rgba(255,255,255,0.85); padding:8px; font:12px sans-serif; line-height:1.3; }
          .legend-item { margin-bottom:4px; white-space:nowrap; }
          .swatch { display:inline-block; width:16px; height:6px; margin-right:6px; vertical-align:middle; border-radius:2px; }
        </style>
        <div class="legend">
          <div class="legend-item"><span class="swatch" style="background:#3388ff;"></span>Regular Edge</div>
          <div class="legend-item"><span class="swatch" style="background:purple;"></span>Shortcut Edge</div>
        </div>
      `;
      return div;
    };
    legend.addTo(map);
  } catch (e) {
    console.error("initCHGraphOnly failed:", e);
  }
}
