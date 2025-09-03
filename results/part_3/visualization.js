// Global variables
let map;
let roadLayer;
let shortcutLayer;
let poiLayer;
let poiPathLayer;
let poiPathUnpackedLayer;
let regularPathLayer;
let regularPathUnpackedLayer;
let startMarker = null;
let endMarker = null;
let graphData = null;
let networkBounds;
let nodeIndex; // Will hold our spatial index
let layerControl; // For toggling layers

// Initialize the map when the page loads
document.addEventListener('DOMContentLoaded', initMap);

function initMap() {
    // Create the map
    map = L.map('map').setView([0, 0], 13);
    
    // Add the tile layer
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
    }).addTo(map);
    
    // Add info panel
    const info = L.control({position: 'topright'});
    
    info.onAdd = function() {
        this._div = L.DomUtil.create('div', 'info-panel');
        this.update();
        
        // Prevent click events from propagating to the map
        L.DomEvent.disableClickPropagation(this._div);
        
        return this._div;
    };

    info.update = function() {
        this._div.innerHTML = `
            <h3>POI Path Finder</h3>
            <p>Click to set start and end points</p>
            <div id="start-point">Start: Not selected</div>
            <div id="end-point">End: Not selected</div>
            <button id="find-path-btn" class="button" disabled>Find Paths</button>
            <button id="reset-btn" class="button">Reset</button>
            <div id="loading" class="loading">Calculating paths...</div>
            <div id="path-info"></div>
            <div class="path-comparison">
                <div id="regular-path-info"></div>
                <div id="poi-path-info"></div>
            </div>
            <div class="layer-control">
                <h4>Map Layers</h4>
                <label>
                    <input type="checkbox" id="toggle-road" checked> Regular Edges
                </label>
                <label>
                    <input type="checkbox" id="toggle-shortcuts" checked> Shortcut Edges
                </label>
                <label>
                    <input type="checkbox" id="toggle-pois" checked> Points of Interest
                </label>
            </div>
        `;
        
        // Add event listeners after the elements are created
        setTimeout(() => {
            document.getElementById('find-path-btn').addEventListener('click', findPath);
            document.getElementById('reset-btn').addEventListener('click', resetMarkers);
            document.getElementById('toggle-road').addEventListener('change', function() {
                if (this.checked) map.addLayer(roadLayer);
                else map.removeLayer(roadLayer);
            });
            document.getElementById('toggle-shortcuts').addEventListener('change', function() {
                if (this.checked) map.addLayer(shortcutLayer);
                else map.removeLayer(shortcutLayer);
            });
            document.getElementById('toggle-pois').addEventListener('change', function() {
                if (this.checked) map.addLayer(poiLayer);
                else map.removeLayer(poiLayer);
            });
        }, 0);
    };
    
    info.addTo(map);
    
    // Add legend
    const legend = L.control({position: 'bottomright'});
    
    legend.onAdd = function() {
        const div = L.DomUtil.create('div', 'info-panel legend');
        div.innerHTML = `
            <h4>Legend</h4>
            <div><i style="background:#3388ff"></i> Regular Edge</div>
            <div><i style="background:#ff3333"></i> Shortcut Edge</div>
            <div><i style="background:#ff7800"></i> POI</div>
            <div><i style="background:#33cc33"></i> POI Path (shortcuts)</div>
            <div><i style="background:#00aa00"></i> POI Path (unpacked)</div>
            <div><i style="background:#ff3333"></i> Regular Path (shortcuts)</div>
            <div><i style="background:#aa0000"></i> Regular Path (unpacked)</div>
            <div><i style="background:green"></i> Start</div>
            <div><i style="background:red"></i> End</div>
        `;

        // Prevent click events from propagating to the map
        L.DomEvent.disableClickPropagation(div);
        return div;
    };
    
    legend.addTo(map);
    
    // Initialize layers
    roadLayer = L.layerGroup();
    shortcutLayer = L.layerGroup();
    poiLayer = L.layerGroup();
    poiPathLayer = L.layerGroup();
    poiPathUnpackedLayer = L.layerGroup();
    regularPathLayer = L.layerGroup();
    regularPathUnpackedLayer = L.layerGroup();
    
    // Add base layers to map
    roadLayer.addTo(map);
    shortcutLayer.addTo(map);
    poiLayer.addTo(map);
    poiPathLayer.addTo(map);
    poiPathUnpackedLayer.addTo(map);
    regularPathLayer.addTo(map);
    regularPathUnpackedLayer.addTo(map);
    
    // Set up layer control for paths
    const overlays = {
        "Regular Path (shortcuts)": regularPathLayer,
        "Regular Path (unpacked)": regularPathUnpackedLayer,
        "POI Path (shortcuts)": poiPathLayer,
        "POI Path (unpacked)": poiPathUnpackedLayer
    };
    
    layerControl = L.control.layers(null, overlays).addTo(map);
    
    // Load the data
    loadData();
    
    // Set up map click handler
    map.on('click', onMapClick);
}

async function loadData() {
    try {
        // Load the visualization data
        const response = await fetch('visualization_data.json');
        if (!response.ok) {
            throw new Error(`Failed to load data: ${response.status} ${response.statusText}`);
        }
        
        graphData = await response.json();
        
        // Build spatial index for nodes
        buildNodeSpatialIndex();
        
        // Draw the road network
        drawRoadNetwork(graphData.graph);
        
        // Draw the POIs
        drawPOIs(graphData.pois);
        
    } catch (error) {
        console.error('Error loading data:', error);
        alert('Failed to load visualization data');
    }
}

function buildNodeSpatialIndex() {
    // Check if RBush is available
    if (typeof rbush !== 'function') {
        console.error('RBush library not loaded!');
        return;
    }
    
    // Create new RBush index
    nodeIndex = new rbush();
    
    // Insert all nodes into the index
    const items = graphData.graph.nodes.map(node => {
        // RBush expects {minX, minY, maxX, maxY} format with optional properties
        return {
            minX: node.lon,
            minY: node.lat,
            maxX: node.lon,
            maxY: node.lat,
            id: node.id,
            node: node
        };
    });
    
    nodeIndex.load(items);
    console.log(`Loaded ${items.length} nodes into spatial index`);
}

function drawRoadNetwork(graph) {
    // Clear the road layers
    roadLayer.clearLayers();
    shortcutLayer.clearLayers();
    
    // Get bounds for the network
    let minLat = Infinity, maxLat = -Infinity;
    let minLon = Infinity, maxLon = -Infinity;
    
    // Process nodes to find bounds
    graph.nodes.forEach(node => {
        minLat = Math.min(minLat, node.lat);
        maxLat = Math.max(maxLat, node.lat);
        minLon = Math.min(minLon, node.lon);
        maxLon = Math.max(maxLon, node.lon);
    });
    
    // Draw regular edges
    if (graph.regular_edges) {
        graph.regular_edges.forEach(edge => {
            const sourceNode = graph.nodes.find(n => n.id === edge.source);
            const targetNode = graph.nodes.find(n => n.id === edge.target);
            
            if (sourceNode && targetNode) {
                L.polyline([
                    [sourceNode.lat, sourceNode.lon],
                    [targetNode.lat, targetNode.lon]
                ], {
                    color: '#3388ff',
                    weight: 1.5,
                    opacity: 0.6
                }).addTo(roadLayer);
            }
        });
    }
    
    // Draw shortcut edges
    if (graph.shortcut_edges) {
        graph.shortcut_edges.forEach(edge => {
            const sourceNode = graph.nodes.find(n => n.id === edge.source);
            const targetNode = graph.nodes.find(n => n.id === edge.target);
            
            if (sourceNode && targetNode) {
                L.polyline([
                    [sourceNode.lat, sourceNode.lon],
                    [targetNode.lat, targetNode.lon]
                ], {
                    color: '#ff3333',
                    weight: 1,
                    opacity: 0.4,
                    dashArray: '3, 3'
                }).addTo(shortcutLayer);
            }
        });
    }
    
    // Set the map bounds
    networkBounds = L.latLngBounds(
        [minLat, minLon],
        [maxLat, maxLon]
    );
    
    map.fitBounds(networkBounds);
}

function drawPOIs(pois) {
    // Clear the POI layer
    poiLayer.clearLayers();
    
    // Draw each POI
    pois.forEach(poi => {
        const poiNode = graphData.graph.nodes.find(n => n.id === poi.id);
        if (poiNode) {
            L.circleMarker([poiNode.lat, poiNode.lon], {
                radius: 6,
                fillColor: '#ff7800',
                color: '#000',
                weight: 1,
                opacity: 1,
                fillOpacity: 0.8
            }).bindTooltip(`POI ${poi.id}`).addTo(poiLayer);
        }
    });
}

function onMapClick(e) {
    if (!graphData) return;
    
    // Find closest node to click
    const closestNode = findClosestNode(e.latlng.lat, e.latlng.lng);
    
    if (!closestNode) return;
    
    if (!startMarker) {
        // Set start marker
        startMarker = L.marker([closestNode.lat, closestNode.lon], {
            icon: createIcon('green'),
            title: 'Start'
        }).bindTooltip(`Start: Node ${closestNode.id}`).addTo(map);
        
        document.getElementById('start-point').textContent = `Start: Node ${closestNode.id}`;
        updateFindButton();
    } else if (!endMarker) {
        // Set end marker
        endMarker = L.marker([closestNode.lat, closestNode.lon], {
            icon: createIcon('red'),
            title: 'End'
        }).bindTooltip(`End: Node ${closestNode.id}`).addTo(map);
        
        document.getElementById('end-point').textContent = `End: Node ${closestNode.id}`;
        updateFindButton();
    }
}

function findClosestNode(lat, lng) {
    if (!graphData) return null;
    
    // Use spatial index if available
    if (nodeIndex) {
        // Search in a small bounding box first
        const searchRadius = 0.01; // Adjust based on your data scale
        const searchBounds = {
            minX: lng - searchRadius,
            minY: lat - searchRadius,
            maxX: lng + searchRadius,
            maxY: lat + searchRadius
        };
        
        // Get candidates within the search bounds
        const candidates = nodeIndex.search(searchBounds);
        
        // If no nodes in the initial search area, expand search
        if (candidates.length === 0) {
            return findClosestNodeLinear(lat, lng);
        }
        
        // Find the closest among candidates
        let closestNode = null;
        let minDist = Infinity;
        
        candidates.forEach(item => {
            const node = item.node;
            const dist = (node.lat - lat) ** 2 + (node.lon - lng) ** 2;
            if (dist < minDist) {
                minDist = dist;
                closestNode = node;
            }
        });
        
        return closestNode;
    } else {
        // Fall back to linear search if index not available
        return findClosestNodeLinear(lat, lng);
    }
}

// Fallback to linear search if spatial index doesn't find candidates
function findClosestNodeLinear(lat, lng) {
    let closestNode = null;
    let minDist = Infinity;
    
    graphData.graph.nodes.forEach(node => {
        const dist = (node.lat - lat) ** 2 + (node.lon - lng) ** 2;
        if (dist < minDist) {
            minDist = dist;
            closestNode = node;
        }
    });
    
    return closestNode;
}

function createIcon(color) {
    return L.divIcon({
        className: 'custom-div-icon',
        html: `<div style="background-color: ${color}; width: 12px; height: 12px; border-radius: 6px; border: 2px solid white;"></div>`,
        iconSize: [12, 12],
        iconAnchor: [6, 6]
    });
}

function updateFindButton() {
    const findPathBtn = document.getElementById('find-path-btn');
    findPathBtn.disabled = !(startMarker && endMarker);
}

async function findPath() {
    if (!startMarker || !endMarker) return;
    
    // Show loading indicator
    const loadingEl = document.getElementById('loading');
    loadingEl.classList.add('visible');
    
    // Clear previous paths
    poiPathLayer.clearLayers();
    poiPathUnpackedLayer.clearLayers();
    regularPathLayer.clearLayers();
    regularPathUnpackedLayer.clearLayers();
    
    document.getElementById('path-info').textContent = '';
    document.getElementById('regular-path-info').textContent = '';
    document.getElementById('poi-path-info').textContent = '';
    
    try {
        // Get start and end node IDs
        const startNode = findClosestNode(startMarker.getLatLng().lat, startMarker.getLatLng().lng);
        const endNode = findClosestNode(endMarker.getLatLng().lat, endMarker.getLatLng().lng);
        
        if (!startNode || !endNode) {
            throw new Error('Invalid start or end node');
        }
        
        // Call the API to find both paths
        const response = await fetch(`/api/compare_paths?start=${startNode.id}&end=${endNode.id}`);
        const result = await response.json();
        
        if (result.status === 'error') {
            document.getElementById('path-info').textContent = result.message || 'Error finding paths';
            return;
        }
        
        // Draw the POI path with shortcuts
        if (result.poi_path && result.poi_path.length > 0) {
            drawPath(result.poi_path, poiPathLayer, '#33cc33', true);
            document.getElementById('poi-path-info').textContent = 
                `Path via POI: Distance ${result.poi_distance.toFixed(2)}, POIs visited: ${result.poi_count}`;
                
            // Draw the unpacked POI path if available
            if (result.poi_path_unpacked && result.poi_path_unpacked.length > 0) {
                drawPath(result.poi_path_unpacked, poiPathUnpackedLayer, '#00aa00', true, 0.6);
            }
        } else {
            document.getElementById('poi-path-info').textContent = 'No valid path containing a POI found';
        }
        
        // Draw the regular path with shortcuts
        if (result.regular_path && result.regular_path.length > 0) {
            drawPath(result.regular_path, regularPathLayer, '#ff3333', false);
            document.getElementById('regular-path-info').textContent = 
                `Regular path: Distance ${result.regular_distance.toFixed(2)}`;
                
            // Draw the unpacked regular path if available
            if (result.regular_path_unpacked && result.regular_path_unpacked.length > 0) {
                drawPath(result.regular_path_unpacked, regularPathUnpackedLayer, '#aa0000', false, 0.6);
            }
        } else {
            document.getElementById('regular-path-info').textContent = 'No regular path found';
        }
        
        // Update the overall path info
        if (result.poi_path && result.regular_path) {
            const diff = result.poi_distance - result.regular_distance;
            const percentage = ((diff / result.regular_distance) * 100).toFixed(2);
            document.getElementById('path-info').textContent = 
                `Path via POI is ${Math.abs(percentage)}% ${diff >= 0 ? 'longer' : 'shorter'} than regular path`;
        }
        
    } catch (error) {
        console.error('Error finding path:', error);
        document.getElementById('path-info').textContent = 'Error finding paths';
    } finally {
        // Hide loading indicator
        loadingEl.classList.remove('visible');
    }
}

function drawPath(path, layerGroup, color, highlightPOIs, opacity = 0.8, weight = 4) {
    if (!path || !path.length) return;
    
    // Create points for the path
    const points = path.map(nodeId => {
        const node = graphData.graph.nodes.find(n => n.id === nodeId);
        if (!node) {
            console.warn(`Node ID ${nodeId} not found in graph data`);
            return null;
        }
        return [node.lat, node.lon];
    }).filter(p => p !== null);
    
    if (points.length < 2) {
        console.warn('Not enough valid points to draw path');
        return;
    }
    
    // Draw the line
    L.polyline(points, {
        color: color,
        weight: weight,
        opacity: opacity
    }).addTo(layerGroup);
    
    // Highlight POIs on the path if requested
    if (highlightPOIs) {
        path.forEach(nodeId => {
            const isPOI = graphData.pois.some(poi => poi.id === nodeId);
            if (isPOI) {
                const node = graphData.graph.nodes.find(n => n.id === nodeId);
                if (node) {
                    L.circleMarker([node.lat, node.lon], {
                        radius: 8,
                        fillColor: '#ff0000',
                        color: '#fff',
                        weight: 2,
                        opacity: 1,
                        fillOpacity: 0.8
                    }).bindTooltip(`POI ${nodeId} (on path)`).addTo(layerGroup);
                }
            }
        });
    }
}

function resetMarkers() {
    // Remove markers
    if (startMarker) {
        map.removeLayer(startMarker);
        startMarker = null;
    }
    
    if (endMarker) {
        map.removeLayer(endMarker);
        endMarker = null;
    }
    
    // Clear paths
    poiPathLayer.clearLayers();
    poiPathUnpackedLayer.clearLayers();
    regularPathLayer.clearLayers();
    regularPathUnpackedLayer.clearLayers();
    
    // Reset UI
    document.getElementById('start-point').textContent = 'Start: Not selected';
    document.getElementById('end-point').textContent = 'End: Not selected';
    document.getElementById('path-info').textContent = '';
    document.getElementById('regular-path-info').textContent = '';
    document.getElementById('poi-path-info').textContent = '';
    document.getElementById('find-path-btn').disabled = true;
}
