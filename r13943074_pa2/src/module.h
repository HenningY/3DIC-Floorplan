#ifndef MODULE_H
#define MODULE_H

#include <vector>
#include <string>
#include <iomanip>
#include <iostream>
#include <algorithm>
using namespace std;

class Contour
{
public:
    // Constructor and destructor
    Contour(size_t x, size_t y) :
        _x(x), _y(y), _prev(NULL), _next(NULL) { }
    ~Contour() { }

    // Basic access methods
    size_t getX() const        { return _x; }
    size_t getY() const        { return _y; }
    Contour* getPrev() const   { return _prev; }
    Contour* getNext() const   { return _next; }
    string getName() const     { return _name; }

    // Set functions
    void setX(size_t x)          { _x = x; }
    void setY(size_t y)          { _y = y; }
    void setPrev(Contour* prev)  { _prev = prev; }
    void setNext(Contour* next)  { _next = next; }
    void setName(string& name)   { _name = name; }

private:
    size_t         _x;    // x coordinate of the contour
    size_t         _y;    // y coordinate of the contour
    Contour*       _prev;  // pointer to the previous node
    Contour*       _next;  // pointer to the next node
    string         _name;  // name of the contour
};

class Terminal
{
public:
    // constructor and destructor
    Terminal(string& name, size_t x, size_t y) :
        _name(name), _x1(x), _y1(y), _x2(x), _y2(y), _orig_x(x), _orig_y(y) { }
    ~Terminal()  { }

    // basic access methods
    const string getName()  { 
        // Remove any line endings from the name
        string cleanName = _name;
        cleanName.erase(std::remove(cleanName.begin(), cleanName.end(), '\r'), cleanName.end());
        cleanName.erase(std::remove(cleanName.begin(), cleanName.end(), '\n'), cleanName.end());
        return cleanName; 
    }
    const size_t getX1()    { return _x1; }
    const size_t getX2()    { return _x2; }
    const size_t getY1()    { return _y1; }
    const size_t getY2()    { return _y2; }

    // set functions
    void setName(string& name) { 
        // Remove any line endings when setting the name
        _name = name;
        _name.erase(std::remove(_name.begin(), _name.end(), '\r'), _name.end());
        _name.erase(std::remove(_name.begin(), _name.end(), '\n'), _name.end());
    }
    void setPos(size_t x1, size_t y1, size_t x2, size_t y2) {
        _x1 = x1;   _y1 = y1;
        _x2 = x2;   _y2 = y2;
    }
    const size_t getOrigX() { return _orig_x; }
    const size_t getOrigY() { return _orig_y; }

protected:
    string      _name;      // module name
    size_t      _x1;        // min x coordinate of the terminal
    size_t      _y1;        // min y coordinate of the terminal
    size_t      _x2;        // max x coordinate of the terminal
    size_t      _y2;        // max y coordinate of the terminal
    size_t      _orig_x;    // original x coordinate of the terminal
    size_t      _orig_y;    // original y coordinate of the terminal
};

class Block : public Terminal
{
public:
    // constructor and destructor
    Block(string& name, size_t w, size_t h, bool rotate = false, size_t dieId = 0) :
        Terminal(name, 0, 0), _w(w), _h(h), _rotate(rotate),
        _contour(new Contour(0, 0)), _dieId(dieId),
        _orig_w(w), _orig_h(h), _tsv_strip_axis(-1) { }
    ~Block() { }

    // basic access methods
    const size_t getWidth()     { return _rotate? _h: _w; }
    const size_t getHeight()    { return _rotate? _w: _h; }
    const size_t getArea()      { return _h * _w; }
    static size_t getMaxX()     { return _maxX; }
    static size_t getMaxY()     { return _maxY; }
    const bool isRotated()      { return _rotate; }
    Contour* getContour()       { return _contour; }

    size_t getDieId() const     { return _dieId; }
    void   setDieId(size_t id)  { _dieId = id; }

    // 直接存取 stored（非 rotate 後）的 w/h
    size_t getRawW() const      { return _w; }
    size_t getRawH() const      { return _h; }

    // TSV inflation 相關
    size_t getOrigW() const          { return _orig_w; }
    size_t getOrigH() const          { return _orig_h; }
    int    getTsvStripAxis() const   { return _tsv_strip_axis; }
    void   setOrigDims(size_t w, size_t h) { _orig_w = w; _orig_h = h; }
    void   setTsvStripAxis(int a)    { _tsv_strip_axis = a; }

    // set functions
    void setWidth(size_t w)             { _w = w; }
    void setHeight(size_t h)            { _h = h; }
    void setRotate()                    { _rotate = !_rotate; }
    static void setMaxX(size_t x)       { _maxX = x; }
    static void setMaxY(size_t y)       { _maxY = y; }
    void setContour(Contour* contour)   { _contour = contour; }


private:
    bool            _rotate;
    size_t          _w;             // stored width  (pre-rotation)
    size_t          _h;             // stored height (pre-rotation)
    static size_t   _maxX;
    static size_t   _maxY;
    Contour*        _contour;
    size_t          _dieId;

    // TSV strip 預留欄位
    size_t          _orig_w;        // 膨脹前的原始 _w
    size_t          _orig_h;        // 膨脹前的原始 _h
    int             _tsv_strip_axis; // 0=strip 在頂(Y+), 1=strip 在右(X+), -1=無 strip
};

class Net
{
public:
    // constructor and destructor
    Net()   { }
    ~Net()  { }

    // basic access methods
    const vector<Terminal*> getTermList()   { return _termList; }

    // modify methods
    void addTerm(Terminal* term) { _termList.push_back(term); }

    // other member functions
    double calcHPWL() {
        double hpwl = 0;
        size_t minX = 1000000, minY = 1000000, maxX = 0, maxY = 0;
        for (auto term : _termList) {
            if ((term->getX1()+term->getX2())/2 > maxX) maxX = (term->getX1()+term->getX2())/2;
            if ((term->getY1()+term->getY2())/2 > maxY) maxY = (term->getY1()+term->getY2())/2;
            if ((term->getX1()+term->getX2())/2 < minX) minX = (term->getX1()+term->getX2())/2;
            if ((term->getY1()+term->getY2())/2 < minY) minY = (term->getY1()+term->getY2())/2;
        }
        hpwl = (maxX - minX + maxY - minY);
        return hpwl;
    }

private:
    vector<Terminal*>   _termList;  // list of terminals the net is connected to
};

struct BStarTreeNode {
    bool rotate;
    Block* block;
    BStarTreeNode* parent;
    BStarTreeNode* left;
    BStarTreeNode* right;

    BStarTreeNode(Block* b)
        : block(b), parent(nullptr), left(nullptr), right(nullptr), rotate(false) {}
        
    BStarTreeNode* getParent() const { return parent; }
    BStarTreeNode* getLeft() const { return left; }
    BStarTreeNode* getRight() const { return right; }
    Block* getBlock() const { return block; }
    bool getRotate() const { return rotate; }
    
    void setParent(BStarTreeNode* p) { parent = p; }
    void setLeft(BStarTreeNode* l) { left = l; }
    void setRight(BStarTreeNode* r) { right = r; }
    void setBlock(Block* b) { block = b; }
    void setRotate(bool r) { rotate = r; }
};

// ── TSV flow 輔助資料結構 ──────────────────────────────────────────────────────

// 浮點 2D bounding box（可累加更新）
struct Bbox {
    double x1, y1, x2, y2;
    bool   valid;

    Bbox() : x1(1e18), y1(1e18), x2(-1e18), y2(-1e18), valid(false) {}

    void update(double cx, double cy) {
        x1 = min(x1, cx); x2 = max(x2, cx);
        y1 = min(y1, cy); y2 = max(y2, cy);
        valid = true;
    }
    double centerX() const { return (x1 + x2) * 0.5; }
    double centerY() const { return (y1 + y2) * 0.5; }
};

// 單一 TSV 候選格點（模組 strip 內的一個 slot）
struct TsvSlot {
    double cx, cy;     // 絕對中心座標（排版後填入）
    size_t die_id;     // 所屬 die（strip 位於 die_id，連到 die_id+1）
    size_t block_idx;  // 對應 Floorplanner::_blocks 的索引
    bool   occupied;   // 是否已被 net 分配

    TsvSlot(double x, double y, size_t d, size_t b)
        : cx(x), cy(y), die_id(d), block_idx(b), occupied(false) {}
};

// 一個 Block 的 TSV strip 描述（排版後計算）
struct TsvStrip {
    size_t block_idx;
    size_t die_id;
    double x1, y1, x2, y2;       // strip 的絕對包圍矩形
    vector<size_t> slot_indices;  // 指向 Floorplanner::_tsv_slots
};

// Net 的 TSV 分配記錄（每個 net × 每個跨越的 tier boundary 各一筆）
struct TsvAssignment {
    size_t net_id;
    size_t tier_lo;   // TSV 連接 die tier_lo 到 die tier_lo+1
    double x, y;      // 分配到的 TSV 中心座標
    size_t slot_idx;  // 對應 _tsv_slots 的索引（SIZE_MAX 表示找不到 slot）
    bool   fallback;  // true = 退而求其次（非 middle zone 內）
};

// ─────────────────────────────────────────────────────────────────────────────

class BStarTree {
    public:
        BStarTree(): _root(nullptr) { }
        ~BStarTree() { deleteTree(_root); }
        
        // Copy constructor
        BStarTree(const BStarTree& other) {
            _root = copyTree(other._root);
        }
        
        // Assignment operator
        BStarTree& operator=(const BStarTree& other) {
            if (this != &other) {
                deleteTree(_root);
                _root = copyTree(other._root);
            }
            return *this;
        }

        BStarTreeNode* getRoot() { return _root; }
        void setRoot(BStarTreeNode* node) {
            _root = node;
            if (node) {
                node->parent = nullptr;
            }
        }
        void insert(BStarTreeNode* parent, BStarTreeNode* node, bool toLeft) {
            node->parent = parent;
            if (toLeft) {
                parent->left = node;
            } else {
                parent->right = node;
            }
        }
        
        // 獲取樹的高度
        int getHeight(BStarTreeNode* node) {
            if (!node) return 0;
            return 1 + max(getHeight(node->left), getHeight(node->right));
        }
        
        // 獲取節點數量
        int getNodeCount(BStarTreeNode* node) {
            if (!node) return 0;
            return 1 + getNodeCount(node->left) + getNodeCount(node->right);
        }
        
        // 檢查樹是否平衡
        bool isBalanced(BStarTreeNode* node) {
            if (!node) return true;
            
            int leftHeight = getHeight(node->left);
            int rightHeight = getHeight(node->right);
            
            // 檢查左右子樹高度差是否不超過1
            if (abs(leftHeight - rightHeight) > 1) return false;
            
            // 遞歸檢查左右子樹
            return isBalanced(node->left) && isBalanced(node->right);
        }
        
        // 獲取所有節點
        vector<BStarTreeNode*> getAllNodes() {
            vector<BStarTreeNode*> nodes;
            getAllNodesHelper(_root, nodes);
            return nodes;
        }

        void printTree(BStarTreeNode* node, int indent = 0) {
            if (!node) return;
            printTree(node->right, indent + 4);
            if (indent > 0) cout << setw(indent) << ' ';
            cout << node->block->getName() << endl;
            printTree(node->left, indent + 4);
        }
        
    private:
        BStarTreeNode* _root;
        
        void deleteTree(BStarTreeNode* node) {
            if (node) {
                deleteTree(node->left);
                deleteTree(node->right);
                delete node;
            }
        };
        
        void getAllNodesHelper(BStarTreeNode* node, vector<BStarTreeNode*>& nodes) {
            if (!node) return;
            nodes.push_back(node);
            getAllNodesHelper(node->left, nodes);
            getAllNodesHelper(node->right, nodes);
        }
        
        // Helper function for deep copy
        BStarTreeNode* copyTree(BStarTreeNode* node) {
            if (!node) return nullptr;
            
            BStarTreeNode* newNode = new BStarTreeNode(node->block);
            newNode->left = copyTree(node->left);
            newNode->right = copyTree(node->right);
            
            // Update parent pointers
            if (newNode->left) newNode->left->parent = newNode;
            if (newNode->right) newNode->right->parent = newNode;
            
            return newNode;
        }
};


#endif  // MODULE_H
