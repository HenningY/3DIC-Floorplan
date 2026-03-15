#ifndef FLOORPLANNER_H
#define FLOORPLANNER_H

#include <fstream>
#include <vector>
#include <string>
#include "module.h"

using namespace std;

class Floorplanner
{
public:
    Floorplanner(fstream& input_blk, fstream& input_net, double alpha,
                 double tsv_size = 3.0):
        _cost(0), _alpha(alpha), _total_area(0), _total_hpwl(0), _penalty(0),
        _avg_area(1), _avg_hpwl(1), _num_dies(1), _tsv_size(tsv_size) {
        parseInput(input_blk, input_net);
    }
    ~Floorplanner(){
        clear();
    }
    void clear();

    void parseInput(fstream& input_blk, fstream& input_net);
    void determineSide();
    void adjustTerminals();
    void generateBalancedBStarTree();
    void floorplan();
    void writeResult(fstream& output);
    void calculateTotalBoundingBox();
    void calculateTotalHPWL();
    void calculatePenalty();
    void calculateCost();
    void exportJSON(const string& filename);
    void updatePosition(BStarTreeNode* node);
    void updateTreePositions(BStarTreeNode* node);
    void simulatedAnnealing();
    void simulatedAnnealingRoundRobin();
    void initCenterLayout();
    double calculateActiveDieCost(size_t active_die);
    double calculateIntraDiePenalty(size_t die_idx);
    void rotateBlock(Block* block);
    void deleteAndInsertNode(BStarTreeNode* node, BStarTreeNode* to_node);
    void swapNodes(BStarTreeNode* node1, BStarTreeNode* node2);
    double getRandomDouble();
    bool acceptNewSolution(double oldCost, double newCost, double temperature);
    void keepRotate();
    void shuffleBlocks();
    void makeSymmetricTerminals();
    void makeSymmetricBlocks();
    void returnOriginalTerminals();

    // ── TSV flow ──────────────────────────────────────────────────────────────
    void   inflateModulesForTsv();
    void   deflateModules();
    void   extractTsvCandidates();
    void   assignTsvToNets();
    double calculateTotalPerDieHPWL();
    // helpers
    Bbox   computeMiddleZone(const Bbox& bd, const Bbox& bd1);
    size_t findSlotInMiddleZone(size_t tier, const Bbox& zone,
                                double pref_cx, double pref_cy);
    size_t findNearestSlot(size_t tier, double pref_cx, double pref_cy);

private:
    // instance
    vector<Block*> _blocks;
    vector<Terminal*> _terminals;
    vector<Net*> _nets;

    // basic information
    size_t _max_x;              // maximum x coordinate for all blocks
    size_t _max_y;              // maximum y coordinate for all blocks
    size_t _num_dies;           // number of dies
    vector<size_t> _max_x_die;  // per-die outline width
    vector<size_t> _max_y_die;  // per-die outline height
    vector<vector<Block*>> _die_blocks; // blocks grouped per die

    bool _xSymmetric;           // whether the floorplan is x-symmetric
    bool _ySymmetric;           // whether the floorplan is y-symmetric
    double _runtime;            // runtime of the floorplan

    // about tree
    BStarTree _bstar_tree;      // current tree
    BStarTree _best_tree;       // best tree found so far
    BStarTree _last_tree;       // record the last tree

    // 3DIC: one B* tree per die
    vector<BStarTree> _bstar_trees; // current trees per die
    vector<BStarTree> _best_trees;  // best trees per die
    vector<BStarTree> _last_trees;  // last trees per die (SA backup)

    // about cost
    double _cost;               // cost of the floorplan
    double _init_cost;          // initial cost of the floorplan
    double _best_cost;          // best cost found so far
    double _alpha;              // scaling factor
    double _total_area;         // total area of the floorplan
    double _total_hpwl;         // total hpwl of the floorplan
    double _avg_area;           // average area of the floorplan
    double _avg_hpwl;           // average hpwl of the floorplan
    double _best_area;          // best area found so far
    double _best_hpwl;          // best hpwl found so far
    double _penalty;            // penalty for the floorplan
    double _best_penalty;       // best penalty found so far

    // about SA
    int _accept_num;            // number of accepted moves when new_cost > old_cost
    double _current_temp;       // current temperature

    // ── TSV flow ──────────────────────────────────────────────────────────────
    double _tsv_size;                         // TSV footprint 邊長（e.g. 3）

    vector<TsvStrip>       _tsv_strips;       // 每個 inflated block 的 strip 描述
    vector<TsvSlot>        _tsv_slots;        // 全域候選格點池
    vector<vector<size_t>> _tier_slot_map;    // [d] = die d→d+1 boundary 的 slot 索引
    vector<TsvAssignment>  _tsv_assignments;  // 最終 TSV 分配結果
    vector<vector<size_t>> _net_tsv_map;      // [net_idx] = assignment 索引列表

    // uint32_t _seed;
};


#endif