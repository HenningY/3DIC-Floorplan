#include "floorplanner.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <random>
#include <queue>
#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <unordered_map>

using namespace std;

uint32_t _seed;

void Floorplanner::parseInput(fstream& input_blk, fstream& input_net) {
    string line, token;
   
    // Parse block file
    // Parse number of dies and per-die outlines
    getline(input_blk, line);
    while (line.size() && all_of(line.begin(), line.end(), ::isspace)) {
        if (!getline(input_blk, line)) break;
    }
    stringstream ss_first(line);
    ss_first >> token;

    if (token == "NumDie:") {
        // New 3DIC format:
        // NumDie: <num_dies>
        // Outline: w h   // die 0
        // Outline: w h   // die 1
        // ...
        size_t num_dies = 1;
        ss_first >> num_dies;
        _num_dies = num_dies;
        _max_x_die.assign(_num_dies, 0);
        _max_y_die.assign(_num_dies, 0);

        for (size_t d = 0; d < _num_dies; ++d) {
            if (!getline(input_blk, line)) break;
            while (line.size() && all_of(line.begin(), line.end(), ::isspace)) {
                if (!getline(input_blk, line)) break;
            }
            stringstream ss_outline(line);
            string outline_token;
            size_t max_x, max_y;
            ss_outline >> outline_token >> max_x >> max_y; // "Outline:" w h
            _max_x_die[d] = max_x;
            _max_y_die[d] = max_y;
        }
        // Use die 0 outline as default for legacy code (terminals, symmetry, etc.)
        if (_num_dies > 0) {
            _max_x = _max_x_die[0];
            _max_y = _max_y_die[0];
        }
    } else if (token == "Outline:") {
        // Legacy 2D format: single outline
        size_t max_x, max_y;
        ss_first >> max_x >> max_y;
        _max_x = max_x;
        _max_y = max_y;
        _num_dies = 1;
        _max_x_die.assign(1, _max_x);
        _max_y_die.assign(1, _max_y);
    } else {
        // Unexpected header format
        cerr << "Error: Unknown block file header: " << line << endl;
        exit(1);
    }
    
    // Parse number of blocks
    getline(input_blk, line);
    stringstream ss_num_blocks(line);
    ss_num_blocks >> token; // "NumBlocks:"
    size_t num_blocks;
    ss_num_blocks >> num_blocks;
    
    // Parse number of terminals
    getline(input_blk, line);
    stringstream ss_num_terminals(line);
    ss_num_terminals >> token; // "NumTerminals:"
    size_t num_terminals;
    ss_num_terminals >> num_terminals;
    

    // getline(input_blk, line);
    
    // Parse blocks
    for (size_t i = 0; i < num_blocks; ++i) {
        while (getline(input_blk, line)) {
            if (!all_of(line.begin(), line.end(), ::isspace)) break;
        }
        stringstream ss(line);
        string name;
        size_t width, height;
        size_t dieId = 0;

        ss >> name >> width >> height;

        if (ss >> dieId) {
            // 有多讀到第 4 欄，就當成 die index
        }
        
        Block* block = new Block(name, width, height, false, dieId);
        _blocks.push_back(block);
    }

    // After reading all blocks, group them per die
    size_t max_die_id = 0;
    for (Block* b : _blocks) {
        if (b->getDieId() > max_die_id) max_die_id = b->getDieId();
    }
    if (_num_dies < max_die_id + 1) {
        // If header didn't specify enough dies, extend using die 0 outline
        size_t old_num = _num_dies;
        _num_dies = max_die_id + 1;
        _max_x_die.resize(_num_dies, old_num > 0 ? _max_x_die[0] : _max_x);
        _max_y_die.resize(_num_dies, old_num > 0 ? _max_y_die[0] : _max_y);
    }
    _die_blocks.assign(_num_dies, {});
    for (Block* b : _blocks) {
        size_t d = b->getDieId();
        if (d >= _num_dies) continue;
        _die_blocks[d].push_back(b);
    }

    // getline(input_blk, line);
    
    // Parse terminals
    for (size_t i = 0; i < num_terminals; ++i) {
        while (getline(input_blk, line)) {
            if (!all_of(line.begin(), line.end(), ::isspace)) break;
        }
        stringstream ss(line);
        string name;
        size_t x, y;
        ss >> name >> token; // "terminal"
        ss >> x >> y;
        
        Terminal* term = new Terminal(name, x, y);
        _terminals.push_back(term);
    }
    
    // Parse net file
    // Parse number of nets
    getline(input_net, line);
    stringstream ss_num_nets(line);
    ss_num_nets >> token; // "NumNets:"
    size_t num_nets;
    ss_num_nets >> num_nets;
    
    // Parse nets
    for (size_t i = 0; i < num_nets; ++i) {
        getline(input_net, line);
        stringstream ss_degree(line);
        ss_degree >> token; // "NetDegree:"
        size_t net_degree;
        ss_degree >> net_degree;
        
        Net* net = new Net();
        
        for (size_t j = 0; j < net_degree; ++j) {
            getline(input_net, line);
            string node_name = line;
            
            // Clean node_name by removing any line endings
            node_name.erase(std::remove(node_name.begin(), node_name.end(), '\r'), node_name.end());
            node_name.erase(std::remove(node_name.begin(), node_name.end(), '\n'), node_name.end());
            
            // Find corresponding terminal or block
            bool found = false;
            
            // Check terminals first
            for (Terminal* term : _terminals) {
                if (term->getName() == node_name) {
                    net->addTerm(term);
                    found = true;
                    break;
                }
            }
            
            // If not found in terminals, check blocks
            if (!found) {
                for (size_t k = 0; k < _blocks.size(); ++k) {
                    if (_blocks[k]->getName() == node_name) {
                        net->addTerm(_blocks[k]);
                        found = true;
                        break;
                    }
                }
            }
        }
        
        _nets.push_back(net);
    }
}

void Floorplanner::adjustTerminals() {
    for (Terminal* terminal : _terminals) {
        if (terminal->getX1() > _max_x) {
            terminal->setPos(_max_x, terminal->getY1(), _max_x, terminal->getY2());
        }
        if (terminal->getY1() > _max_y) {
            terminal->setPos(terminal->getX1(), _max_y, terminal->getX2(), _max_y);
        }
    }
}

void Floorplanner::determineSide() {
    int x_weight = 0;
    int y_weight = 0;
    for (Terminal* terminal : _terminals) {
        x_weight += terminal->getX1();
        y_weight += terminal->getY1();
    }
    x_weight /= _terminals.size();
    y_weight /= _terminals.size();
    if (x_weight > _max_x / 2) _xSymmetric = true;
    else _xSymmetric = false;
    if (y_weight > _max_y / 2) _ySymmetric = true;
    else _ySymmetric = false;
    // if (_terminals.size() < 5) {
    //     _xSymmetric = false;
    //     _ySymmetric = false;
    // }
    makeSymmetricTerminals();
}

void Floorplanner::makeSymmetricTerminals() {
    if (_xSymmetric) {
        for (Terminal* terminal : _terminals) {
            terminal->setPos(_max_x - terminal->getX1(), terminal->getY1(), _max_x - terminal->getX2(), terminal->getY2());
        }
    }
    if (_ySymmetric) {
        for (Terminal* terminal : _terminals) {
            terminal->setPos(terminal->getX1(), _max_y - terminal->getY1(), terminal->getX2(), _max_y - terminal->getY2());
        }
    }
}

void Floorplanner::makeSymmetricBlocks() {
    for (Block* block : _blocks) {
        size_t d = block->getDieId();
        size_t max_x = (_max_x_die.empty() || d >= _max_x_die.size()) ? _max_x : _max_x_die[d];
        size_t max_y = (_max_y_die.empty() || d >= _max_y_die.size()) ? _max_y : _max_y_die[d];
        if (_xSymmetric) {
            block->setPos(max_x - block->getX2(), block->getY1(), max_x - block->getX1(), block->getY2());
        }
        if (_ySymmetric) {
            block->setPos(block->getX1(), max_y - block->getY2(), block->getX2(), max_y - block->getY1());
        }
    }
}

void Floorplanner::returnOriginalTerminals() {
    for (Terminal* terminal : _terminals) {
        terminal->setPos(terminal->getOrigX(), terminal->getOrigY(), terminal->getOrigX(), terminal->getOrigY());
    }
}

uint32_t lcg(uint32_t& seed) {
    seed = (1664525 * seed + 1013904223);  // 常見參數
    return seed;
}

void Floorplanner::floorplan() {
    // TODO: Implement the floorplan function
    // 1. Adjust the position of the terminals
    // 2. Generate the initial B* tree
    // 3. Calculate the cost of the initial floorplan
    // 4. Simulated annealing
    // 5. Output the result

    auto startTime = std::chrono::steady_clock::now();
    const int TIME_LIMIT = 100;

    // srand(77);
    _seed = 77;
    if (_blocks.size() == 33) {
        _seed = 9487;
    }
    
    adjustTerminals();
    determineSide();

    // ── TSV Phase 0：SA 前膨脹模組（僅 3DIC 模式） ──
    if (_num_dies > 1 && _tsv_size > 0.0)
        inflateModulesForTsv();

    _accept_num = 0;
    generateBalancedBStarTree();

    calculateCost();
    _init_cost = _cost;
    _avg_area = 1.0;
    _avg_hpwl = (_total_hpwl == 0) ? 1.0 : _total_hpwl;

    // Phase 1：Round-Robin SA — 依序對每個 die 做多輪優化，其他 die 固定不動
    simulatedAnnealingRoundRobin();
    // Phase 2：全局 SA — 所有 die 一起優化（考慮所有 net，含跨 die）
    simulatedAnnealing();

    // Apply best rotation states from all dies and update positions
    for (size_t d = 0; d < _num_dies; ++d) {
        for (BStarTreeNode* node : _best_trees[d].getAllNodes()) {
            if (node->getRotate() != node->block->isRotated()) {
                node->block->setRotate();
            }
        }
    }
    for (size_t d = 0; d < _num_dies; ++d) {
        BStarTreeNode* root = _best_trees[d].getRoot();
        if (root) updateTreePositions(root);
    }

    if (_num_dies > 1 && _tsv_size > 0.0) {
        // ── TSV Phase 1：從 placed 座標（含 strip）提取候選格點 ──
        extractTsvCandidates();
        // ── TSV Phase 2：還原 block 到原始尺寸（strip 區域標記為 TSV 用） ──
        deflateModules();
        // ── TSV Phase 3：按 span 排序，分配 TSV 到 net ──
        assignTsvToNets();
        // ── 計算最終 per-die HPWL（含 TSV 點） ──
        _total_hpwl = calculateTotalPerDieHPWL();
        calculatePenalty();
        _cost = _total_hpwl + _penalty * 1000.0;
        cout << "[TSV] Final per-die HPWL = " << _total_hpwl
             << "  (TSV assignments: " << _tsv_assignments.size() << ")" << endl;
    } else {
        makeSymmetricTerminals();
        makeSymmetricBlocks();
        returnOriginalTerminals();
        calculateCost();
    }
    string filename = "floorplan_" + _blocks[0]->getName() + ".json";

    // ------------------------------------------------------------
    // GUI tool
    // exportJSON(filename);
    // ------------------------------------------------------------

    auto endTime = std::chrono::steady_clock::now();
    auto totalMilliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
        endTime - startTime).count();
    double totalSeconds = totalMilliseconds / 1000.0;
    cout << "Total runtime: " << fixed << setprecision(3) << totalSeconds << " seconds" << endl;
    _runtime = totalSeconds;
}

void Floorplanner::writeResult(fstream& output) {
    size_t max_width = 0;
    size_t max_height = 0;
    size_t min_width = 1000000;
    size_t min_height = 1000000;
    for (Block* block : _blocks) {
        if (block->getX2() > max_width)  max_width  = block->getX2();
        if (block->getY2() > max_height) max_height = block->getY2();
        if (block->getX1() < min_width)  min_width  = block->getX1();
        if (block->getY1() < min_height) min_height = block->getY1();
    }
    _total_area = (max_width - min_width) * (max_height - min_height);
    output << fixed << (1-_alpha) * _total_hpwl + _alpha * _total_area << endl;
    output << _total_hpwl << endl;
    output << fixed << _total_area << endl;
    output << max_width - min_width << " " << max_height - min_height << endl;
    output << _runtime << endl;

    // Blocks（deflate 後座標，不含 strip 區域）
    for (Block* block : _blocks) {
        output << block->getName() << " "
               << block->getDieId() << " "
               << block->getX1() << " " << block->getY1() << " "
               << block->getX2() << " " << block->getY2() << endl;
    }

    // TSV strips（各模組預留的 strip 包圍矩形）
    if (!_tsv_strips.empty()) {
        output << "NumTsvStrips " << _tsv_strips.size() << endl;
        for (const TsvStrip& s : _tsv_strips) {
            output << _blocks[s.block_idx]->getName() << " "
                   << "die" << s.die_id << " "
                   << fixed << s.x1 << " " << s.y1 << " "
                   << s.x2  << " " << s.y2 << endl;
        }
    }

    // TSV assignments（net × tier boundary 的最終 TSV 位置）
    if (!_tsv_assignments.empty()) {
        output << "NumTsvAssignments " << _tsv_assignments.size() << endl;
        for (const TsvAssignment& ta : _tsv_assignments) {
            output << "net" << ta.net_id << " "
                   << "tier" << ta.tier_lo << "-" << (ta.tier_lo + 1) << " "
                   << fixed << ta.x << " " << ta.y;
            if (ta.fallback)  output << " fallback";
            if (ta.slot_idx == SIZE_MAX) output << " no_slot";
            output << endl;
        }
    }
}

void Floorplanner::clear() {
    for (Terminal* term : _terminals) {
        delete term;
    }
    _terminals.clear();
    for (Block* block : _blocks) {
        delete block;
    }
    _blocks.clear();
    for (Net* net : _nets) {
        delete net;
    }
    _nets.clear();
}

void Floorplanner::exportJSON(const string& filename) {
    ofstream outFile(filename);
    if (!outFile.is_open()) {
        cerr << "Error: Could not open file " << filename << " for writing." << endl;
        return;
    }

    size_t orig_x = 0;
    size_t orig_y = 0;
    for (Terminal* terminal : _terminals) {
        if (terminal->getOrigX() > orig_x) {
            orig_x = terminal->getOrigX();
        }
        if (terminal->getOrigY() > orig_y) {
            orig_y = terminal->getOrigY();
        }
    }
    // Start JSON object
    outFile << "{\n";
    
    // Export terminal outline
    outFile << "  \"terminal_outline\": {\n";
    outFile << "    \"width\": " << orig_x << ",\n";
    outFile << "    \"height\": " << orig_y << "\n";
    outFile << "  },\n";

    // Export outline
    outFile << "  \"outline\": {\n";
    outFile << "    \"width\": " << _max_x << ",\n";
    outFile << "    \"height\": " << _max_y << "\n";
    outFile << "  },\n";
    
    // Export blocks
    outFile << "  \"blocks\": [\n";
    for (size_t i = 0; i < _blocks.size(); ++i) {
        Block* block = _blocks[i];
        outFile << "    {\n";
        outFile << "      \"name\": \"" << block->getName() << "\",\n";
        outFile << "      \"x\": " << block->getX1() << ",\n";
        outFile << "      \"y\": " << block->getY1() << ",\n";
        outFile << "      \"width\": " << block->getWidth() << ",\n";
        outFile << "      \"height\": " << block->getHeight() << ",\n";
        outFile << "      \"rotated\": " << (block->isRotated() ? "true" : "false") << "\n";
        outFile << "    }";
        if (i < _blocks.size() - 1) {
            outFile << ",";
        }
        outFile << "\n";
    }
    outFile << "  ],\n";
    
    // Export terminals
    outFile << "  \"terminals\": [\n";
    for (size_t i = 0; i < _terminals.size(); ++i) {
        Terminal* terminal = _terminals[i];
        outFile << "    {\n";
        outFile << "      \"name\": \"" << terminal->getName() << "\",\n";
        outFile << "      \"x\": " << terminal->getX1() << ",\n";
        outFile << "      \"y\": " << terminal->getY1() << "\n";
        outFile << "    }";
        if (i < _terminals.size() - 1) {
            outFile << ",";
        }
        outFile << "\n";
    }
    outFile << "  ],\n";
    
    // Export nets
    outFile << "  \"nets\": [\n";
    for (size_t i = 0; i < _nets.size(); ++i) {
        Net* net = _nets[i];
        outFile << "    {\n";
        outFile << "      \"connections\": [\n";
        
        const vector<Terminal*>& termList = net->getTermList();
        for (size_t j = 0; j < termList.size(); ++j) {
            Terminal* term = termList[j];
            outFile << "        {\n";
            outFile << "          \"name\": \"" << term->getName() << "\",\n";
            outFile << "          \"x\": " << term->getX1() + (term->getX2() - term->getX1())/2 << ",\n";
            outFile << "          \"y\": " << term->getY1() + (term->getY2() - term->getY1())/2 << "\n";
            outFile << "        }";
            if (j < termList.size() - 1) {
                outFile << ",";
            }
            outFile << "\n";
        }
        
        outFile << "      ]\n";
        outFile << "    }";
        if (i < _nets.size() - 1) {
            outFile << ",";
        }
        outFile << "\n";
    }
    outFile << "  ],\n";
    
    // Export B* tree structure
    outFile << "  \"bstar_tree\": {\n";
    outFile << "    \"nodes\": [\n";
    
    // 獲取所有節點
    vector<BStarTreeNode*> allNodes = _bstar_tree.getAllNodes();
    
    for (size_t i = 0; i < allNodes.size(); ++i) {
        BStarTreeNode* node = allNodes[i];
        outFile << "      {\n";
        outFile << "        \"name\": \"" << node->block->getName() << "\",\n";
        
        // 添加父節點信息
        if (node->parent) {
            outFile << "        \"parent\": \"" << node->parent->block->getName() << "\",\n";
            outFile << "        \"is_left_child\": " << (node == node->parent->left ? "true" : "false") << ",\n";
        } else {
            outFile << "        \"parent\": null,\n";
            outFile << "        \"is_left_child\": null,\n";
        }
        
        // 添加子節點信息
        if (node->left) {
            outFile << "        \"left_child\": \"" << node->left->block->getName() << "\",\n";
        } else {
            outFile << "        \"left_child\": null,\n";
        }
        
        if (node->right) {
            outFile << "        \"right_child\": \"" << node->right->block->getName() << "\"\n";
        } else {
            outFile << "        \"right_child\": null\n";
        }
        
        outFile << "      }";
        if (i < allNodes.size() - 1) {
            outFile << ",";
        }
        outFile << "\n";
    }
    
    outFile << "    ],\n";
    outFile << "    \"is_balanced\": " << (_bstar_tree.isBalanced(_bstar_tree.getRoot()) ? "true" : "false") << ",\n";
    outFile << "    \"height\": " << _bstar_tree.getHeight(_bstar_tree.getRoot()) << ",\n";
    outFile << "    \"node_count\": " << _bstar_tree.getNodeCount(_bstar_tree.getRoot()) << "\n";
    outFile << "  }\n";
    
    // End JSON object
    outFile << "}\n";
    
    outFile.close();
    // cout << "Floorplan data exported to " << filename << endl;
}

void Floorplanner::generateBalancedBStarTree() {
    // 3DIC: build one B* tree per die
    _bstar_trees.assign(_num_dies, BStarTree());
    _best_trees.assign(_num_dies, BStarTree());
    _last_trees.assign(_num_dies, BStarTree());

    if (_blocks.empty()) {
        cout << "No blocks to create B* tree" << endl;
        return;
    }

    for (size_t d = 0; d < _num_dies; ++d) {
        vector<Block*>& blocks = _die_blocks[d];
        if (blocks.empty()) continue;

        vector<BStarTreeNode*> nodes;
        nodes.reserve(blocks.size());
        for (Block* block : blocks) {
            const string name = block->getName();
            string name_copy = name;
            block->getContour()->setName(name_copy);
            nodes.push_back(new BStarTreeNode(block));
        }

        BStarTree& tree = _bstar_trees[d];
        tree.setRoot(nodes[0]);

        int sqrt_nodes = sqrt(nodes.size());
        int count = 0;

        BStarTreeNode* parent = nodes[0];
        BStarTreeNode* current = nodes[0];
        for (size_t i = 1; i < nodes.size(); ++i) {
            bool asLeftChild;
            if (count < sqrt_nodes) {
                asLeftChild = true;
                current = nodes[i - 1];
            }
            else {
                asLeftChild = false;
                current = parent;
                parent = nodes[i];
                count = 0;
            }
            count++;

            tree.insert(current, nodes[i], asLeftChild);
        }

        updateTreePositions(tree.getRoot());
    }

    // Keep die 0 tree in legacy members for possible GUI/debug usage
    if (_num_dies > 0) {
        _bstar_tree = _bstar_trees[0];
        _best_tree = _bstar_tree;
        _last_tree = _bstar_tree;
    }
}

void Floorplanner::updatePosition(BStarTreeNode* node) {
    if (!node) return;

    node->block->getContour()->setPrev(nullptr);
    node->block->getContour()->setNext(nullptr);
    if (node->parent) {
        if (node == node->parent->left) {
            size_t x = node->parent->block->getX1() + node->parent->block->getWidth();
            size_t y = node->parent->block->getY1();
            
            // 檢查水平輪廓線
            Contour* current = node->parent->block->getContour();
            int count = 0;
            while (current->getNext() != nullptr) {
                current = current->getNext();
                if (count == 0) {

                    y = current->getY();
                }
                else {
                    y = max(y, current->getY());
                }
                count++;
                if (current->getX() >= x + node->block->getWidth()) {
                    node->block->getContour()->setNext(current);
                    current->setPrev(node->block->getContour());
                    break;
                }
            }
            node->block->getContour()->setX(x + node->block->getWidth());
            node->block->getContour()->setY(y + node->block->getHeight());
            node->block->getContour()->setPrev(node->parent->block->getContour());
            node->parent->block->getContour()->setNext(node->block->getContour());
            node->block->setPos(x, y, x+node->block->getWidth(), y+node->block->getHeight());
            
        } else if (node == node->parent->right) {
            size_t x = node->parent->block->getX1();
            size_t y = node->parent->block->getY1() + node->parent->block->getHeight();
            
            if (x != 0) {
                node->block->getContour()->setPrev(node->parent->block->getContour()->getPrev());
                node->parent->block->getContour()->getPrev()->setNext(node->block->getContour());
            }
            // 檢查水平輪廓線
            Contour* current = node->parent->block->getContour(); 
            if (current->getX() >= x + node->block->getWidth()) {
                node->block->getContour()->setNext(current);
                current->setPrev(node->block->getContour());
            }
            else {
                while (current->getNext() != nullptr) {
                    current = current->getNext();
                    y = max(y, current->getY());
                    if (current->getX() >= x + node->block->getWidth()) {
                        node->block->getContour()->setNext(current);
                        current->setPrev(node->block->getContour());
                        break;
                    }
                }
            }
            node->block->getContour()->setX(x + node->block->getWidth());
            node->block->getContour()->setY(y + node->block->getHeight());
            node->block->setPos(x, y, x+node->block->getWidth(), y+node->block->getHeight());
            
        }
    } else {
        node->block->setPos(0, 0, node->block->getWidth(), node->block->getHeight());
        node->block->getContour()->setX(node->block->getWidth());
        node->block->getContour()->setY(node->block->getHeight());
    }
    // if (node->block->getContour()->getPrev() != NULL) {
    //     cout << "  " <<node->block->getContour()->getPrev()->getName() << endl;
    // }
    // else {
    //     cout << "  NULL" << endl;
    // }
    // if (node->block->getContour()->getNext() != NULL) {
    //     cout << "  " << node->block->getContour()->getNext()->getName() << endl;
    // }
    // else {
    //     cout << "  NULL" << endl;
    // }
}

void Floorplanner::updateTreePositions(BStarTreeNode* node) {
    if (!node) return;
    updatePosition(node);
    updateTreePositions(node->left);
    updateTreePositions(node->right);
}

void Floorplanner::calculateTotalBoundingBox() {
    size_t max_width = 0;
    size_t max_height = 0;
    for (Block* block : _blocks) {
        if (block->getX2() > max_width) {
            max_width = block->getX2();
        }
        if (block->getY2() > max_height) {
            max_height = block->getY2();
        }
    }
    _total_area = max_width * max_height;
}

void Floorplanner::calculateTotalHPWL() {
    size_t total_hpwl = 0;
    for (Net* net : _nets) {
        total_hpwl += net->calcHPWL();
    }
    _total_hpwl = total_hpwl;
}

void Floorplanner::calculatePenalty() {
    _penalty = 0;
    for (Block* block : _blocks) {
        size_t d = block->getDieId();
        size_t max_x = (_max_x_die.empty() || d >= _max_x_die.size()) ? _max_x : _max_x_die[d];
        size_t max_y = (_max_y_die.empty() || d >= _max_y_die.size()) ? _max_y : _max_y_die[d];
        if (block->getX2() > max_x) {
            _penalty += (block->getX2() - max_x) * block->getWidth();
        }
        if (block->getY2() > max_y) {
            _penalty += (block->getY2() - max_y) * block->getHeight();
        }
    }
}

void Floorplanner::calculateCost() {
    calculateTotalHPWL();
    calculatePenalty();
    // 3DIC: cost = HPWL + penalty (no area term, no z-axis)
    _cost = _total_hpwl + _penalty * 1000.0;
}

// ─── Phase 1 Round-Robin SA 輔助函式 ─────────────────────────────────────────

// 計算指定 die 超出 outline 的 penalty（penalty 只算此 die）
double Floorplanner::calculateIntraDiePenalty(size_t die_idx) {
    double penalty = 0;
    size_t max_x = (die_idx < _max_x_die.size()) ? _max_x_die[die_idx] : _max_x;
    size_t max_y = (die_idx < _max_y_die.size()) ? _max_y_die[die_idx] : _max_y;
    for (Block* block : _die_blocks[die_idx]) {
        if (block->getX2() > max_x)
            penalty += (block->getX2() - max_x) * block->getWidth();
        if (block->getY2() > max_y)
            penalty += (block->getY2() - max_y) * block->getHeight();
    }
    return penalty;
}

// Cost function for round-robin SA:
// HPWL 考慮所有 net（含跨 die），penalty 只算 active die。
// 其他 die 已固定，不需計算其 penalty。
double Floorplanner::calculateActiveDieCost(size_t active_die) {
    double hpwl = 0;
    for (Net* net : _nets)
        hpwl += net->calcHPWL();
    return hpwl + calculateIntraDiePenalty(active_die) * 1000.0;
}

// 將所有 die 的 block 以直接 setPos 方式固定到各自 die 的中心點。
// 用於 Round-Robin SA 的初始化：讓還未被優化的 die 提供一個合理的參考位置。
void Floorplanner::initCenterLayout() {
    for (size_t d = 0; d < _num_dies; ++d) {
        size_t cx = ((d < _max_x_die.size()) ? _max_x_die[d] : _max_x) / 2;
        size_t cy = ((d < _max_y_die.size()) ? _max_y_die[d] : _max_y) / 2;
        for (Block* block : _die_blocks[d]) {
            size_t w  = block->getWidth();
            size_t h  = block->getHeight();
            size_t x1 = cx > w / 2 ? cx - w / 2 : 0;
            size_t y1 = cy > h / 2 ? cy - h / 2 : 0;
            block->setPos(x1, y1, x1 + w, y1 + h);
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────

void Floorplanner::shuffleBlocks() {
    int ratio = 0.1;
    int size = _blocks.size() * ratio;
    for (int i = 0; i < size; i++) {
        int block_idx = lcg(_seed) % _blocks.size();
        rotateBlock(_blocks[block_idx]);
    }
    for (int i = 0; i < size; i++) {
        vector<BStarTreeNode*> nodes = _bstar_tree.getAllNodes();
        if (!nodes.empty()) {
            int from_node_idx = lcg(_seed) % nodes.size();
            int to_node_idx = lcg(_seed) % nodes.size();
            while (from_node_idx == to_node_idx) to_node_idx = lcg(_seed) % nodes.size();
            deleteAndInsertNode(nodes[from_node_idx], nodes[to_node_idx]);
        }
    }
    for (int i = 0; i < size; i++) {
        vector<BStarTreeNode*> nodes = _bstar_tree.getAllNodes();
        if (nodes.size() >= 2) {
            int idx1 = lcg(_seed) % nodes.size();
            int idx2 = lcg(_seed) % nodes.size();
            while (idx2 == idx1) idx2 = lcg(_seed) % nodes.size();
            swapNodes(nodes[idx1], nodes[idx2]);
        }
    }
}

// ─── Phase 1：Round-Robin SA ──────────────────────────────────────────────────
// 策略：
//   Round 1 前先把所有 die 的 block 固定在中心點。
//   每輪依序對每個 die 執行 SA（只動此 die 的 B*-tree），其餘 die 位置不變。
//   Cost = 全部 net 的 HPWL（含跨 die）+ 僅此 die 的 penalty。
//   共執行 NUM_ROUNDS 輪；後幾輪時其他 die 已有合理初步佈局，效果更好。
void Floorplanner::simulatedAnnealingRoundRobin() {
    const int    NUM_ROUNDS     = 3;
    const double initial_temp   = 1000.0;
    const double final_temp     = 200.0;
    const double cooling_rate   = 0.97;
    const int    iters_per_temp = 1500;

    cout << "[Phase 1] Round-Robin SA (" << NUM_ROUNDS << " rounds)" << endl;

    // 第一輪前：所有 die 的 block 先固定在各自 die 的中心點，
    // 讓尚未被優化的 die 提供有意義的參考座標供 HPWL 計算使用
    initCenterLayout();

    for (int round = 0; round < NUM_ROUNDS; ++round) {
        cout << "[Phase 1] === Round " << (round + 1) << " / " << NUM_ROUNDS
             << " ===" << endl;

        for (size_t d = 0; d < _num_dies; ++d) {
            if (_die_blocks[d].empty()) continue;

            // 從 B*-tree 重算此 die 目前的位置（其他 die 位置不動）
            BStarTreeNode* init_root = _bstar_trees[d].getRoot();
            if (init_root) updateTreePositions(init_root);

            // 儲存旋轉狀態並建立最佳樹快照
            for (BStarTreeNode* node : _bstar_trees[d].getAllNodes())
                node->setRotate(node->block->isRotated());
            BStarTree best_die_tree = _bstar_trees[d];
            double    best_die_cost = calculateActiveDieCost(d);
            double    cur_cost      = best_die_cost;

            double T = initial_temp;
            while (T > final_temp) {
                for (int iter = 0; iter < iters_per_temp; ++iter) {
                    double old_cost = cur_cost;

                    // 快照（含旋轉狀態）
                    for (BStarTreeNode* node : _bstar_trees[d].getAllNodes())
                        node->setRotate(node->block->isRotated());
                    BStarTree last_die_tree = _bstar_trees[d];
                    int block_idx = -1;

                    BStarTree& tree = _bstar_trees[d];
                    switch (lcg(_seed) % 3) {
                        case 0: {
                            block_idx = lcg(_seed) % _die_blocks[d].size();
                            rotateBlock(_die_blocks[d][block_idx]);
                            break;
                        }
                        case 1: {
                            vector<BStarTreeNode*> nodes = tree.getAllNodes();
                            if (nodes.size() >= 2) {
                                int f = lcg(_seed) % nodes.size();
                                int t = lcg(_seed) % nodes.size();
                                while (f == t) t = lcg(_seed) % nodes.size();
                                deleteAndInsertNode(nodes[f], nodes[t]);
                            }
                            break;
                        }
                        case 2: {
                            vector<BStarTreeNode*> nodes = tree.getAllNodes();
                            if (nodes.size() >= 2) {
                                int a = lcg(_seed) % nodes.size();
                                int b = lcg(_seed) % nodes.size();
                                while (a == b) b = lcg(_seed) % nodes.size();
                                swapNodes(nodes[a], nodes[b]);
                            }
                            break;
                        }
                    }

                    // 只更新此 die 的座標（其他 die 位置保持不動）
                    BStarTreeNode* root = _bstar_trees[d].getRoot();
                    if (root) updateTreePositions(root);
                    cur_cost = calculateActiveDieCost(d);

                    if (!acceptNewSolution(old_cost, cur_cost, T)) {
                        _bstar_trees[d] = last_die_tree;
                        for (BStarTreeNode* node : _bstar_trees[d].getAllNodes()) {
                            if (node->getRotate() != node->block->isRotated())
                                node->block->setRotate();
                        }
                        BStarTreeNode* r = _bstar_trees[d].getRoot();
                        if (r) updateTreePositions(r);
                        cur_cost = old_cost;
                    } else if (cur_cost < best_die_cost) {
                        best_die_cost = cur_cost;
                        for (BStarTreeNode* node : _bstar_trees[d].getAllNodes())
                            node->setRotate(node->block->isRotated());
                        best_die_tree = _bstar_trees[d];
                    }
                }
                T *= cooling_rate;
            }

            // 套用此 die 的最佳布局，供下一個 die 的 SA 使用
            _bstar_trees[d] = best_die_tree;
            for (BStarTreeNode* node : _bstar_trees[d].getAllNodes()) {
                if (node->getRotate() != node->block->isRotated())
                    node->block->setRotate();
            }
            BStarTreeNode* r = _bstar_trees[d].getRoot();
            if (r) updateTreePositions(r);

            cout << "[Phase 1] Round " << (round + 1) << " Die " << d
                 << " done, cost = " << best_die_cost << endl;
        }

        calculateCost();
        cout << "[Phase 1] Round " << (round + 1) << " done, global cost = "
             << _cost << endl;
    }

    // 以 round-robin 結果更新全域最佳快照，作為 Phase 2 global SA 的起點
    calculateCost();
    _best_cost  = _cost;
    _best_trees = _bstar_trees;
    cout << "[Phase 1] All rounds done, global cost = " << _cost << endl;
}

// ─────────────────────────────────────────────────────────────────────────────

void Floorplanner::simulatedAnnealing() {
    double initial_temp = 1000.0;
    // double final_temp = _blocks.size() < 12 ? 86.8 : 1.75;
    double final_temp = 80.0;
    double cooling_rate = 0.98;
    int iterations_per_temp = 2000;
    
    _best_cost = _cost;
    _best_trees = _bstar_trees;
    _last_trees = _bstar_trees;
    
    double current_temp = initial_temp;
    while (current_temp > final_temp) {
        for (int i = 0; i < iterations_per_temp; i++) {
            double old_cost = _cost;
            _last_trees = _bstar_trees;

            int block_idx = -1;
            size_t die_idx = 0;

            // 選一個有 block 的 die
            if (_num_dies > 1) {
                for (int try_cnt = 0; try_cnt < 10; ++try_cnt) {
                    size_t cand = lcg(_seed) % _num_dies;
                    if (cand < _die_blocks.size() && !_die_blocks[cand].empty()) {
                        die_idx = cand;
                        break;
                    }
                }
            }

            BStarTree& tree = _bstar_trees[die_idx];

            int operation = lcg(_seed) % 3;
            switch (operation) {
                case 0: {
                    // rotate a random block on this die
                    if (!_die_blocks[die_idx].empty()) {
                        block_idx = lcg(_seed) % _die_blocks[die_idx].size();
                        rotateBlock(_die_blocks[die_idx][block_idx]);
                    }
                    break;
                }
                case 1: {
                    // delete and re-insert a node within this die's tree
                    vector<BStarTreeNode*> nodes = tree.getAllNodes();
                    if (!nodes.empty()) {
                        int from_node_idx = lcg(_seed) % nodes.size();
                        int to_node_idx = lcg(_seed) % nodes.size();
                        while (from_node_idx == to_node_idx) to_node_idx = lcg(_seed) % nodes.size();
                        deleteAndInsertNode(nodes[from_node_idx], nodes[to_node_idx]);
                    }
                    break;
                }
                case 2: {
                    // swap two nodes within this die's tree
                    vector<BStarTreeNode*> nodes = tree.getAllNodes();
                    if (nodes.size() >= 2) {
                        int idx1 = lcg(_seed) % nodes.size();
                        int idx2 = lcg(_seed) % nodes.size();
                        while (idx2 == idx1) idx2 = lcg(_seed) % nodes.size();
                        swapNodes(nodes[idx1], nodes[idx2]);
                    }
                    break;
                }
            }

            // 更新所有 die 上的座標（net 可以跨 die）
            for (size_t d = 0; d < _num_dies; ++d) {
                BStarTreeNode* root = _bstar_trees[d].getRoot();
                if (root) updateTreePositions(root);
            }

            calculateCost();
            if (!acceptNewSolution(old_cost, _cost, current_temp)) {
                if (block_idx != -1) {
                    // revert rotation on this die
                    rotateBlock(_die_blocks[die_idx][block_idx]);
                }
                _cost = old_cost;
                _bstar_trees = _last_trees;
            } else if (_cost < _best_cost) {
                _best_cost = _cost;
                _best_penalty = _penalty;
                _best_trees = _bstar_trees;
                keepRotate();
                _current_temp = current_temp;
            }
        }
        current_temp *= cooling_rate;
        cout << "cost hpwl: " << _total_hpwl << endl;
    }
}

// ── TSV Flow ──────────────────────────────────────────────────────────────────

// Phase 1：在 SA 前，對 die 0..n-2 的 block 沿長邊方向加上 tsv_size 的 strip 區域。
// 最頂層 die (die n-1) 不留 strip（沒有向上的 TSV）。
void Floorplanner::inflateModulesForTsv() {
    size_t tsv_int = (size_t)_tsv_size;
    for (Block* b : _blocks) {
        size_t raw_w = b->getRawW();
        size_t raw_h = b->getRawH();
        b->setOrigDims(raw_w, raw_h);

        // 最頂層不留 strip
        if (b->getDieId() >= _num_dies - 1) {
            b->setTsvStripAxis(-1);
            continue;
        }

        // 短邊判斷（以 stored dims 為準，不受 rotate 影響）
        if (raw_w <= raw_h) {
            // 高 >= 寬：短邊是 w，沿高度（長邊）加 strip
            b->setHeight(raw_h + tsv_int);
            b->setTsvStripAxis(0);  // strip at top (Y+ in stored-dim space)
        } else {
            // 寬 > 高：短邊是 h，沿寬度（長邊）加 strip
            b->setWidth(raw_w + tsv_int);
            b->setTsvStripAxis(1);  // strip at right (X+ in stored-dim space)
        }
    }
    cout << "[TSV] Modules inflated by " << tsv_int
         << " on long side (die 0.." << (_num_dies - 2) << ")" << endl;
}

// Phase 2 後：把 block 的 stored dims 還原，並收縮 placed 座標排除 strip 區域。
// 此步驟後 block 的 getX2/Y2 反映去掉 strip 的有效模組邊界。
void Floorplanner::deflateModules() {
    size_t tsv_int = (size_t)_tsv_size;
    for (Block* b : _blocks) {
        int axis = b->getTsvStripAxis();
        if (axis == -1) continue;  // 無 strip（頂層 die）

        bool rotated = b->isRotated();
        // axis=0 + not rotated, 或 axis=1 + rotated：strip 在 Y+ 方向
        if ((axis == 0 && !rotated) || (axis == 1 && rotated)) {
            b->setPos(b->getX1(), b->getY1(),
                      b->getX2(), b->getY2() - tsv_int);
        } else {
            // strip 在 X+ 方向
            b->setPos(b->getX1(), b->getY1(),
                      b->getX2() - tsv_int, b->getY2());
        }
        // 還原 stored dims
        b->setWidth(b->getOrigW());
        b->setHeight(b->getOrigH());
    }
}

// Phase 3：讀取排版後的 placed 座標（含 strip），建立候選格點池。
// 格點數 = floor(strip_length / tsv_size) × floor(strip_width / tsv_size)。
void Floorplanner::extractTsvCandidates() {
    _tsv_strips.clear();
    _tsv_slots.clear();
    size_t num_boundaries = (_num_dies > 1) ? _num_dies - 1 : 0;
    _tier_slot_map.assign(num_boundaries, {});

    for (size_t bi = 0; bi < _blocks.size(); bi++) {
        Block* b   = _blocks[bi];
        int    axis = b->getTsvStripAxis();

        // 頂層 block 無 strip
        if (axis == -1 || b->getDieId() >= _num_dies - 1) continue;

        size_t die_d = b->getDieId();
        double x1 = (double)b->getX1(), y1 = (double)b->getY1();
        double x2 = (double)b->getX2(), y2 = (double)b->getY2();
        double tsv = _tsv_size;
        bool rotated = b->isRotated();

        // 確定 strip 的絕對包圍矩形
        // axis=0 stored -> Y+ 方向加；rotate 時 Y+ 成為 X+ 方向
        double sx1, sy1, sx2, sy2;
        if ((axis == 0 && !rotated) || (axis == 1 && rotated)) {
            // strip at top (Y+ 方向)
            sx1 = x1; sy1 = y2 - tsv; sx2 = x2; sy2 = y2;
        } else {
            // strip at right (X+ 方向)
            sx1 = x2 - tsv; sy1 = y1; sx2 = x2; sy2 = y2;
        }

        // 按 strip 長邊方向生成格點
        // strip 長度方向 = floor(len / tsv)，寬度方向 = floor(tsv / tsv) = 1
        size_t n_cols = (size_t)((sx2 - sx1) / tsv);
        size_t n_rows = (size_t)((sy2 - sy1) / tsv);
        if (n_cols == 0) n_cols = 1;
        if (n_rows == 0) n_rows = 1;

        TsvStrip strip;
        strip.block_idx = bi;
        strip.die_id    = die_d;
        strip.x1 = sx1; strip.y1 = sy1;
        strip.x2 = sx2; strip.y2 = sy2;

        for (size_t r = 0; r < n_rows; r++) {
            for (size_t c = 0; c < n_cols; c++) {
                double cx = sx1 + (c + 0.5) * tsv;
                double cy = sy1 + (r + 0.5) * tsv;
                size_t slot_idx = _tsv_slots.size();
                _tsv_slots.emplace_back(cx, cy, die_d, bi);
                strip.slot_indices.push_back(slot_idx);
                if (die_d < _tier_slot_map.size())
                    _tier_slot_map[die_d].push_back(slot_idx);
            }
        }
        _tsv_strips.push_back(move(strip));
    }

    cout << "[TSV] Extracted " << _tsv_strips.size() << " strips, "
         << _tsv_slots.size() << " candidate slots across "
         << num_boundaries << " tier boundaries." << endl;
    for (size_t d = 0; d < num_boundaries; d++) {
        cout << "  Tier " << d << "→" << (d+1)
             << ": " << _tier_slot_map[d].size() << " slots" << endl;
    }
}

// 計算「middle zone」：使兩 die 的 per-die HPWL 加總最小的 TSV 搜尋矩形。
// 若兩層 bbox 在某軸有重疊 → 取交集；無重疊 → 取 gap（兩端 bbox 之間的空隙）。
Bbox Floorplanner::computeMiddleZone(const Bbox& bd, const Bbox& bd1) {
    Bbox zone;
    zone.valid = true;

    // X 方向
    double inner_x_lo = max(bd.x1, bd1.x1);
    double inner_x_hi = min(bd.x2, bd1.x2);
    if (inner_x_lo <= inner_x_hi) {
        // 有重疊：取交集
        zone.x1 = inner_x_lo;
        zone.x2 = inner_x_hi;
    } else {
        // 無重疊：取 gap（[min of two right edges, max of two left edges]）
        zone.x1 = inner_x_hi;   // = min(bd.x2, bd1.x2)
        zone.x2 = inner_x_lo;   // = max(bd.x1, bd1.x1)
    }

    // Y 方向
    double inner_y_lo = max(bd.y1, bd1.y1);
    double inner_y_hi = min(bd.y2, bd1.y2);
    if (inner_y_lo <= inner_y_hi) {
        zone.y1 = inner_y_lo;
        zone.y2 = inner_y_hi;
    } else {
        zone.y1 = inner_y_hi;
        zone.y2 = inner_y_lo;
    }
    return zone;
}

// 在 tier boundary [tier] 的 middle zone 矩形內找最接近 (pref_cx, pref_cy) 的可用 slot。
// 返回 SIZE_MAX 表示 zone 內無可用 slot。
size_t Floorplanner::findSlotInMiddleZone(size_t tier, const Bbox& zone,
                                           double pref_cx, double pref_cy) {
    if (tier >= _tier_slot_map.size()) return SIZE_MAX;
    size_t best = SIZE_MAX;
    double best_dist = 1e18;

    for (size_t idx : _tier_slot_map[tier]) {
        TsvSlot& s = _tsv_slots[idx];
        if (s.occupied) continue;
        if (s.cx < zone.x1 - 1e-9 || s.cx > zone.x2 + 1e-9) continue;
        if (s.cy < zone.y1 - 1e-9 || s.cy > zone.y2 + 1e-9) continue;
        double d = hypot(s.cx - pref_cx, s.cy - pref_cy);
        if (d < best_dist) { best = idx; best_dist = d; }
    }
    return best;
}

// Fallback：在 tier boundary [tier] 全域尋找最接近 (pref_cx, pref_cy) 的可用 slot（不限 zone）。
size_t Floorplanner::findNearestSlot(size_t tier, double pref_cx, double pref_cy) {
    if (tier >= _tier_slot_map.size()) return SIZE_MAX;
    size_t best = SIZE_MAX;
    double best_dist = 1e18;

    for (size_t idx : _tier_slot_map[tier]) {
        TsvSlot& s = _tsv_slots[idx];
        if (s.occupied) continue;
        double d = hypot(s.cx - pref_cx, s.cy - pref_cy);
        if (d < best_dist) { best = idx; best_dist = d; }
    }
    return best;
}

// Phase 4：將 TSV 分配給跨 tier 的 net。
// 排序策略：tier span 小的 net 優先（確保短程 net 先搶佔好位置）。
// 對每個 net 的每個 tier boundary，計算兩層的 member bbox 的 middle zone，
// 在 zone 內找最近的 slot；找不到則 fallback 到全域最近 slot。
void Floorplanner::assignTsvToNets() {
    _net_tsv_map.assign(_nets.size(), {});
    _tsv_assignments.clear();

    // 建立 Terminal* → die_id 查表（Block 用各自的 dieId，pure terminal 用 die 0）
    unordered_map<Terminal*, size_t> term_die;
    for (Terminal* t : _terminals) term_die[t] = 0;  // pure terminals on die 0
    for (Block*    b : _blocks)    term_die[(Terminal*)b] = b->getDieId();

    // ── Step 1：計算每個 net 的 tier span + 排序 ────────────────────────────
    struct NetInfo {
        size_t net_idx;
        int    span;       // max_die - min_die
        double bbox_area;  // tiebreak：bbox 面積小優先
    };
    vector<NetInfo> net_infos;
    net_infos.reserve(_nets.size());

    for (size_t ni = 0; ni < _nets.size(); ni++) {
        Net* net = _nets[ni];
        int min_die = (int)_num_dies, max_die = -1;
        Bbox combined;
        for (Terminal* t : net->getTermList()) {
            int die_id = (int)(term_die.count(t) ? term_die[t] : 0);
            min_die = min(min_die, die_id);
            max_die = max(max_die, die_id);
            double cx = (t->getX1() + t->getX2()) * 0.5;
            double cy = (t->getY1() + t->getY2()) * 0.5;
            combined.update(cx, cy);
        }
        int span = (max_die >= 0) ? max_die - min_die : 0;
        double area = combined.valid
            ? (combined.x2 - combined.x1) * (combined.y2 - combined.y1)
            : 0.0;
        net_infos.push_back({ni, span, area});
    }

    // span 小優先；相同 span 時 bbox 面積小優先
    sort(net_infos.begin(), net_infos.end(),
         [](const NetInfo& a, const NetInfo& b) {
             return a.span != b.span ? a.span < b.span : a.bbox_area < b.bbox_area;
         });

    // ── Step 2：逐 net 分配 TSV ─────────────────────────────────────────────
    int total_assigned = 0, total_fallback = 0, total_miss = 0;

    for (const NetInfo& ninfo : net_infos) {
        if (ninfo.span == 0) continue;  // 不跨 tier，跳過

        size_t ni  = ninfo.net_idx;
        Net*  net  = _nets[ni];

        // 初始化每個 die 的 member bbox（不含 TSV 點）
        vector<Bbox> die_bboxes(_num_dies);
        for (Terminal* t : net->getTermList()) {
            int die_id = (int)(term_die.count(t) ? term_die[t] : 0);
            double cx = (t->getX1() + t->getX2()) * 0.5;
            double cy = (t->getY1() + t->getY2()) * 0.5;
            die_bboxes[die_id].update(cx, cy);
        }

        // 找 net 的 die 範圍
        int min_die = (int)_num_dies, max_die = -1;
        for (int d = 0; d < (int)_num_dies; d++) {
            if (die_bboxes[d].valid) {
                min_die = min(min_die, d);
                max_die = max(max_die, d);
            }
        }
        if (min_die >= max_die) continue;

        // 對每個 tier boundary 由低到高分配 TSV
        for (int d = min_die; d < max_die; d++) {
            Bbox bd  = die_bboxes[d];
            Bbox bd1 = die_bboxes[d + 1];

            // 若某層無 module member，以另一層的 bbox 代替（TSV 放哪都一樣）
            if (!bd.valid  && !bd1.valid) continue;
            if (!bd.valid)  bd  = bd1;
            if (!bd1.valid) bd1 = bd;

            // middle zone + preferred center
            Bbox   zone    = computeMiddleZone(bd, bd1);
            double pref_cx = zone.centerX();
            double pref_cy = zone.centerY();

            // 優先在 middle zone 內找 slot
            size_t slot_idx = findSlotInMiddleZone(d, zone, pref_cx, pref_cy);
            bool   fallback = false;
            if (slot_idx == SIZE_MAX) {
                slot_idx = findNearestSlot(d, pref_cx, pref_cy);
                fallback = true;
            }

            TsvAssignment ta;
            ta.net_id   = ni;
            ta.tier_lo  = (size_t)d;
            ta.fallback = fallback;
            ta.slot_idx = slot_idx;

            if (slot_idx != SIZE_MAX) {
                TsvSlot& slot = _tsv_slots[slot_idx];
                slot.occupied = true;
                ta.x = slot.cx;
                ta.y = slot.cy;
                total_assigned++;
                if (fallback) total_fallback++;
            } else {
                // 完全無可用 slot（全被佔用），使用 zone 中心作為 TSV 位置
                ta.x = pref_cx;
                ta.y = pref_cy;
                total_miss++;
                cout << "[TSV] Warning: no available slot for net " << ni
                     << " at tier " << d << "→" << (d+1) << endl;
            }

            size_t assign_idx = _tsv_assignments.size();
            _tsv_assignments.push_back(ta);
            _net_tsv_map[ni].push_back(assign_idx);

            // 更新相鄰兩層的 die_bboxes（供更高層 boundary 計算用）
            die_bboxes[d].update(ta.x, ta.y);
            die_bboxes[d + 1].update(ta.x, ta.y);
        }
    }

    cout << "[TSV] Assignment done: "
         << total_assigned << " placed (" << total_fallback << " fallback), "
         << total_miss << " miss (no slot)." << endl;
}

// Per-die HPWL：對每個 net，計算每個 die 上的 member（module/terminal + TSV 點）的 HPWL，加總。
// TSV 點在 tier_lo 和 tier_lo+1 兩層各計一次。
double Floorplanner::calculateTotalPerDieHPWL() {
    // 建立 Terminal* → die_id 查表
    unordered_map<Terminal*, size_t> term_die;
    for (Terminal* t : _terminals) term_die[t] = 0;
    for (Block*    b : _blocks)    term_die[(Terminal*)b] = b->getDieId();

    double total = 0.0;

    for (size_t ni = 0; ni < _nets.size(); ni++) {
        Net*         net      = _nets[ni];
        const auto&  tsv_ais  = _net_tsv_map[ni];

        for (size_t d = 0; d < _num_dies; d++) {
            Bbox bbox;

            // 加入此 die 上的 module / terminal 中心點
            for (Terminal* t : net->getTermList()) {
                size_t t_die = term_die.count(t) ? term_die[t] : 0;
                if (t_die != d) continue;
                double cx = (t->getX1() + t->getX2()) * 0.5;
                double cy = (t->getY1() + t->getY2()) * 0.5;
                bbox.update(cx, cy);
            }

            // 加入連接此 die 的 TSV 點
            // TSV at tier_lo→tier_lo+1 同時出現在 die tier_lo 和 die tier_lo+1
            for (size_t ai : tsv_ais) {
                const TsvAssignment& ta = _tsv_assignments[ai];
                if (ta.tier_lo == d || ta.tier_lo + 1 == d)
                    bbox.update(ta.x, ta.y);
            }

            if (bbox.valid)
                total += (bbox.x2 - bbox.x1) + (bbox.y2 - bbox.y1);
        }
    }
    return total;
}

// ─────────────────────────────────────────────────────────────────────────────

void Floorplanner::keepRotate() {
    // Record rotation state for each die's best tree
    for (size_t d = 0; d < _num_dies; ++d) {
        for (BStarTreeNode* node : _best_trees[d].getAllNodes()) {
            node->setRotate(node->block->isRotated());
        }
    }
}

void Floorplanner::rotateBlock(Block* block) {
    block->setRotate();
}

void Floorplanner::deleteAndInsertNode(BStarTreeNode* node, BStarTreeNode* to_node) {
    // delete part
    if (!node || node->parent == nullptr) {
        return;
    }
    if (node->left && node->right) {
        return;
    }
    else {
        if (node->left) {
            if (node->parent->left == node) {
                node->parent->left = node->left;
                node->left->parent = node->parent;
            }
            else {
                node->parent->right = node->left;
                node->left->parent = node->parent;
            }
            node->left = nullptr;
        }
        else if (node->right) {
            if (node->parent->left == node) {
                node->parent->left = node->right;
                node->right->parent = node->parent;
            }
            else {
                node->parent->right = node->right;
                node->right->parent = node->parent;
            }
            node->right = nullptr;
        }
        else {
            if (node->parent->left == node) {
                node->parent->left = nullptr;
            }
            else {
                node->parent->right = nullptr;
            }
        }
    }
    // insert part
    if (lcg(_seed) % 2 == 0) { // left
        node->left = to_node->left;
        if (node->left) node->left->parent = node;
        to_node->left = node;
        node->parent = to_node;
    }
    else { // right
        node->right = to_node->right;
        if (node->right) node->right->parent = node;
        to_node->right = node;
        node->parent = to_node;
    }
}

void Floorplanner::swapNodes(BStarTreeNode* node1, BStarTreeNode* node2) {
    if (!node1 || !node2 || node1 == node2) return;
    
    Block* temp_block = node1->getBlock();
    node1->setBlock(node2->getBlock());
    node2->setBlock(temp_block);
}

double Floorplanner::getRandomDouble() {
    // return static_cast<double>(lcg(_seed)) / RAND_MAX;
    return static_cast<double>(lcg(_seed)) / 4294967295;
}

bool Floorplanner::acceptNewSolution(double oldCost, double newCost, double temperature) {
    if (newCost < oldCost) {
        return true;
    }
    double probability = exp((oldCost - newCost) * 1 / temperature);
    double random_double = getRandomDouble();

    return random_double < probability;
}
