# Floorplanning

## Compilation

To compile the program, run:

```
make
```

## Execution

To execute the FM algorithm, run:

```
./bin/fp <alpha> <input_block_file> <input_net_file> <output_file>
```

Where:
- `<input_block_file>`: Path to the input block file
- `<input_net_file>`: Path to the input net file
- `<output_file>`: Path where the output will be saved

## Example

```
./bin/fp 0.5 ../input_pa2/ami33.block ../input_pa2/ami33.nets ../output_pa2/ami33_50.out
```



# Floorplan Visualizer (GUI)

This tool allows you to visualize floorplan data including blocks, terminals, and nets.

## Setup and Usage

### Step 1: Export JSON data from your floorplanner

Uncommnet this line to your C++ code so you can export the floorplan:

```floorplan.cpp```
```
void Floorplanner::floorplan() {
    ...
    // GUI tool
    exportJSON(filename);
    ...
}
```

This will generate a JSON file that contains all the floorplan data.

### Step 2: Start the visualization server

#### Option A: Local Usage
Simply open the `visualizer.html` file in your web browser. This works locally without needing any server.

#### Option B: Remote Server Usage
If you're working on a remote server, use the provided Python HTTP server:

2. Run the server script:
   ```bash
   python3 start_server.py
   ```
3. By default, the server will listen on all interfaces (0.0.0.0) on port 8000
4. To specify a different port:
   ```bash
   python3 start_server.py --port 8080
   ```
5. To connect from your local machine to the remote server:
     open `http://localhost:8000/visualizer.html` in your local browser

### Step 3: Load your floorplan data

1. Click the case you want to observe
2. The floorplan will be displayed on the canvas

## Features

- **Toggle Visibility**: Show/hide blocks, terminals, nets, and labels
- **Information Panel**: View details about the floorplan (outline size, number of blocks, etc.)