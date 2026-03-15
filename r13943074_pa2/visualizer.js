// Global variables
let canvas, ctx;
let floorplanData = null;
let scale = 1.0;
let showBlocks = true;
let showTerminals = true;
let showNets = true;
let showLabels = true;
let padding = 50; // Padding around the canvas in pixels
let colors = {};

// Initialize the canvas and event listeners when the page loads
document.addEventListener('DOMContentLoaded', function() {
    canvas = document.getElementById('floorplanCanvas');
    ctx = canvas.getContext('2d');
    
    // Set up event listeners
    setupEventListeners();
    
    // Display initial message
    setCanvasMessage('choose the case to visualize the floorplan');
});

// Set up all event listeners
function setupEventListeners() {
    // File input change listener
    document.getElementById('loadButton1').addEventListener('click', loadJSONFile1);
    document.getElementById('loadButton2').addEventListener('click', loadJSONFile2);
    document.getElementById('loadButton3').addEventListener('click', loadJSONFile3);
    document.getElementById('loadButton4').addEventListener('click', loadJSONFile4);
    document.getElementById('loadButton5').addEventListener('click', loadJSONFile5);
    
    // Toggle buttons
    document.getElementById('toggle-blocks').addEventListener('click', function() {
        this.classList.toggle('active');
        showBlocks = this.classList.contains('active');
        redraw();
    });
    
    document.getElementById('toggle-terminals').addEventListener('click', function() {
        this.classList.toggle('active');
        showTerminals = this.classList.contains('active');
        redraw();
    });
    
    document.getElementById('toggle-nets').addEventListener('click', function() {
        this.classList.toggle('active');
        showNets = this.classList.contains('active');
        redraw();
    });
    
    document.getElementById('toggle-labels').addEventListener('click', function() {
        this.classList.toggle('active');
        showLabels = this.classList.contains('active');
        redraw();
    });
    
    // Scale slider
    document.getElementById('scale-slider').addEventListener('input', function() {
        scale = parseFloat(this.value);
        document.getElementById('scale-value').textContent = scale.toFixed(1) + 'x';
        redraw();
    });
}

// Load JSON file
function loadJSONFile1() {
    console.log("loadJSONFile 1");
    fetch('floorplan_bk1.json')
        .then(response => response.json())
        .then(data => {
            floorplanData = data;
            updateInfoPanel();
            redraw();
        });
    const currentFile = document.querySelector('.current-file');
    currentFile.textContent = 'Current floorplan: ami33';
}
function loadJSONFile2() {
    console.log("loadJSONFile 2");
    fetch('floorplan_M001.json')
        .then(response => response.json())
        .then(data => {
            floorplanData = data;
            updateInfoPanel();
            redraw();
        });
    const currentFile = document.querySelector('.current-file');
    currentFile.textContent = 'Current floorplan: ami49';
}
function loadJSONFile3() {
    console.log("loadJSONFile 3");
    fetch('floorplan_cc_11.json')
        .then(response => response.json())
        .then(data => {
            floorplanData = data;
            updateInfoPanel();
            redraw();
        });
    const currentFile = document.querySelector('.current-file');
    currentFile.textContent = 'Current floorplan: apte';
}
function loadJSONFile4() {
    console.log("loadJSONFile 4");
    fetch('floorplan_clkc.json')
        .then(response => response.json())
        .then(data => {
            floorplanData = data;
            updateInfoPanel();
            redraw();
        });
    const currentFile = document.querySelector('.current-file');
    currentFile.textContent = 'Current floorplan: hp';
}
function loadJSONFile5() {
    console.log("loadJSONFile 5");
    fetch('floorplan_BLKB.json')
        .then(response => response.json())
        .then(data => {
            floorplanData = data;
            updateInfoPanel();
            redraw();
        });
    const currentFile = document.querySelector('.current-file');
    currentFile.textContent = 'Current floorplan: xerox';
}

// Update the information panel with details about the floorplan
function updateInfoPanel() {
    if (!floorplanData) return;
    
    const infoContent = document.getElementById('info-content');
    
    let html = `
        <p><strong>Outline:</strong> ${floorplanData.outline.width} x ${floorplanData.outline.height}</p>
        <p><strong>Blocks:</strong> ${floorplanData.blocks.length}</p>
        <p><strong>Terminals:</strong> ${floorplanData.terminals.length}</p>
        <p><strong>Nets:</strong> ${floorplanData.nets.length}</p>
    `;
    
    infoContent.innerHTML = html;
}

// Draw the floorplan on the canvas
function redraw() {
    if (!floorplanData) return;
    
    // Resize the canvas to fit the floorplan with padding
    let width = floorplanData.terminal_outline.width * scale + 2*padding;
    let height = floorplanData.terminal_outline.height * scale + 2*padding;
    width = Math.max(width, floorplanData.outline.width * scale + 2*padding);
    height = Math.max(height, floorplanData.outline.height * scale + 2*padding);
    canvas.width = width;
    canvas.height = height;
    
    // Clear the canvas
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    
    // Draw the outline
    ctx.strokeStyle = '#000000';
    ctx.lineWidth = 2;
    ctx.strokeRect(
        padding, 
        canvas.height - padding - floorplanData.outline.height * scale, 
        floorplanData.outline.width * scale, 
        floorplanData.outline.height * scale
    );
    
    // Draw grid lines (optional)
    drawGrid();
    
    // Draw nets (if enabled)
    if (showNets) {
        drawNets();
    }
    
    // Draw blocks (if enabled)
    if (showBlocks) {
        drawBlocks();
    }
    
    // Draw terminals (if enabled)
    if (showTerminals) {
        drawTerminals();
    }
}

// Draw grid lines on the canvas
function drawGrid() {
    const gridSize = 100; // Size of grid cells in layout units
    const gridColor = '#f0f0f0';
    
    ctx.strokeStyle = gridColor;
    ctx.lineWidth = 0.5;
    
    // Draw vertical grid lines
    for (let x = 0; x <= floorplanData.outline.width; x += gridSize) {
        ctx.beginPath();
        ctx.moveTo(padding + x * scale, canvas.height - padding);
        ctx.lineTo(padding + x * scale, canvas.height - padding - floorplanData.outline.height * scale);
        ctx.stroke();
    }
    
    // Draw horizontal grid lines
    for (let y = 0; y <= floorplanData.outline.height; y += gridSize) {
        ctx.beginPath();
        ctx.moveTo(padding, canvas.height - padding - y * scale);
        ctx.lineTo(padding + floorplanData.outline.width * scale, canvas.height - padding - y * scale);
        ctx.stroke();
    }
}

// Draw blocks on the canvas
function drawBlocks() {
    floorplanData.blocks.forEach(block => {
        // Get a consistent color for each block
        const blockColor = getColorForName(block.name);
        
        // Calculate y position with y-axis pointing upwards
        const yPos = canvas.height - padding - block.y * scale - block.height * scale;
        
        // Draw the block rectangle
        ctx.fillStyle = blockColor;
        ctx.fillRect(
            padding + block.x * scale,
            yPos,
            block.width * scale,
            block.height * scale
        );
        
        // Draw the block outline
        ctx.strokeStyle = '#000000';
        ctx.lineWidth = 1;
        ctx.strokeRect(
            padding + block.x * scale,
            yPos,
            block.width * scale,
            block.height * scale
        );
        
        // Draw the block name if labels are enabled
        if (showLabels) {
            ctx.fillStyle = '#000000';
            ctx.font = '12px Arial';
            ctx.fillText(
                block.name,
                padding + block.x * scale + 5,
                yPos + 15
            );
        }
    });
}

// Draw terminals on the canvas
function drawTerminals() {
    const radius = 3; // Radius of terminal points
    
    floorplanData.terminals.forEach(terminal => {
        // Calculate y position with y-axis pointing upwards
        const yPos = canvas.height - padding - terminal.y * scale;
        
        // Draw the terminal point
        ctx.beginPath();
        ctx.arc(
            padding + terminal.x * scale,
            yPos,
            radius,
            0,
            Math.PI * 2
        );
        ctx.fillStyle = '#FF0000';
        ctx.fill();
        ctx.strokeStyle = '#000000';
        ctx.lineWidth = 1;
        ctx.stroke();
        
        // Draw the terminal name if labels are enabled
        if (showLabels) {
            ctx.fillStyle = '#000000';
            ctx.font = '10px Arial';
            ctx.fillText(
                terminal.name,
                padding + terminal.x * scale + 5,
                yPos - 5
            );
        }
    });
}

// Draw nets on the canvas
function drawNets() {
    floorplanData.nets.forEach((net, index) => {
        if (!net.connections || net.connections.length < 2) return;
        
        // Get a consistent color for each net
        const netColor = getNetColor(index);
        
        ctx.strokeStyle = netColor;
        ctx.lineWidth = 1;
        
        // Draw lines between all connections
        const connections = net.connections;
        
        for (let i = 0; i < connections.length - 1; i++) {
            const curr = connections[i];
            const next = connections[i+1];
            
            // Calculate y positions with y-axis pointing upwards
            const currY = canvas.height - padding - curr.y * scale;
            const nextY = canvas.height - padding - next.y * scale;
            
            ctx.beginPath();
            ctx.moveTo(padding + curr.x * scale, currY);
            ctx.lineTo(padding + next.x * scale, nextY);
            ctx.stroke();
        }
    });
}

// Get a consistent color for a name
function getColorForName(name) {
    if (!colors[name]) {
        // Generate a pastel color
        const hue = Math.random() * 360;
        const saturation = 50 + Math.random() * 30;
        const lightness = 70 + Math.random() * 10;
        colors[name] = `hsla(${hue}, ${saturation}%, ${lightness}%, 0.7)`;
    }
    
    return colors[name];
}

// Get a color for a net
function getNetColor(index) {
    const hue = (index * 137) % 360; // Use golden angle to distribute colors
    return `hsla(${hue}, 70%, 50%, 0.3)`;
}

// Display a message on the canvas
function setCanvasMessage(message) {
    // Resize canvas to a default size
    canvas.width = 800;
    canvas.height = 500;
    
    // Clear canvas
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    
    // Draw message
    ctx.fillStyle = '#000000';
    ctx.font = '16px Arial';
    ctx.textAlign = 'center';
    ctx.fillText(message, canvas.width / 2, canvas.height / 2);
    ctx.textAlign = 'start'; // Reset text alignment
} 