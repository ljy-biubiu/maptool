#ifndef __DIJKSTRA_H__
#define __DIJKSTRA_H__

#include <iostream>
#include <limits>
#include <vector>
#include <tuple>
#include <algorithm>

namespace cti{
namespace test{

double const DOUBLE_MAX = std::numeric_limits<double>::max();
double const DOUBLE_MIN = std::numeric_limits<double>::min();
uint32_t const UINT32_T_MAX = std::numeric_limits<uint32_t>::max();

typedef struct {
    int    start_id;
    int    end_id;
    double distance;
}DNode;

class EdgeNode{
public:
    EdgeNode(int key, double weight)
    {
        this->key = key;
        this->weight = weight;
        this->next = NULL;
    }
    int key;
    double weight;
    EdgeNode *next;
};

class Graph{
public:
    Graph();
    Graph(uint32_t num=-1);
    ~Graph();
    void insert_edge(uint32_t start, uint32_t end, double weight, bool directed = false);
    void stop_edge(uint32_t start, uint32_t end, bool directed = false);
    std::tuple<double, std::vector<uint32_t>> dijkstra(uint32_t start, uint32_t end);
    bool isRepeat(uint32_t start, uint32_t end);
    inline uint32_t currentNodeNum(){
        return this->G_MAX;
    }
    void print_Graph();
    struct DijkstraData {
        uint32_t parent;
        double distance;
        bool visit;
        DijkstraData(){
            visit = false;
            distance = DOUBLE_MAX;
            parent  = -1;
        }
    };
    bool findNeighborsFromID(int id, std::vector<DNode> &nodes);
private:
    uint32_t G_MAX{10000};
    EdgeNode **edges;
    void dijkstra(EdgeNode **edges, DijkstraData *data, uint32_t start);
    void getShortPath(uint32_t end, const DijkstraData *data, std::vector<uint32_t> &path);
};

Graph::Graph()
{
    this->edges = new EdgeNode*[this->G_MAX];
    for(int i = 0; i < this->G_MAX; i++){
        this->edges[i] = NULL;
    }
}

Graph::Graph(uint32_t num)
{
    if(num > 0 && num < this->G_MAX){
        this->G_MAX = num;
    }
    this->edges = new EdgeNode*[this->G_MAX];
    for(int i = 0; i < this->G_MAX; i++){
        this->edges[i] = NULL;
    }
}

Graph::~Graph()
{
    for (int i = 0; i < this->G_MAX; i++) { 
        if(this->edges[i])
            delete this->edges[i]; 
    } 
    delete this->edges;
}
//判断是否重复
bool Graph::isRepeat(uint32_t start, uint32_t end)
{
    bool ret = false;
    EdgeNode *curr = this->edges[start];
    while(curr != NULL){
        if(curr->key == end){
            ret = true;
            break;
        }
        curr = curr->next;
    }
    return ret;
}

void Graph::insert_edge(uint32_t start, uint32_t end, double weight, bool directed)
{
    if(start >= 0 && start < this->G_MAX && end >= 0 && end < this->G_MAX)
    {
        if(!isRepeat(start,end)){
            EdgeNode *edge = new EdgeNode(end, weight);
            edge->next = this->edges[start];
            this->edges[start] = edge;
        }
        //是否有方向
        if(!directed){
            insert_edge(end, start, weight, true);
        }
    }else{
        std::cerr << "ERRO:Insert Edge Param Erro!(start:" << start << " end:" << end << ")" << std::endl;
    }
}

void Graph::stop_edge(uint32_t start, uint32_t end, bool directed) 
{ 
    if(start >= 0 && start < this->G_MAX && end >= 0 && end < this->G_MAX) 
    {
        if(this->edges[start] != NULL)
        {
            EdgeNode *pre_node = NULL;
            EdgeNode *curr = this->edges[start];
            while(curr != NULL)
            {
                if(curr->key == end)
                {
                    if(pre_node != NULL){
                        pre_node->next = curr->next;
                    }else{
                        this->edges[start] = curr->next;
                    }
                    delete curr; 
                    break;
                }
                pre_node = curr;
                curr = curr->next;
            }
        }
        //是否有方向 
        if(!directed){ 
            stop_edge(end, start, true); 
        }
    }else{
        std::cerr << "ERRO:Stop Edge Param Erro!(start:" << start << " end:" << end << ")" << std::endl;
    }
}

void Graph::getShortPath(uint32_t end, const DijkstraData *data, std::vector<uint32_t> &path)
{
    if(end >= 0 && end < this->G_MAX && data[end].parent != -1){
        getShortPath(data[end].parent, data, path);
        path.push_back(data[end].parent);
    }
}

std::tuple<double, std::vector<uint32_t>> Graph::dijkstra(uint32_t start, uint32_t end)
{
    std::vector<uint32_t> path;
    path.clear();
    double distance = -1;
    if(start >= 0 && start < this->G_MAX && end >= 0 && end < this->G_MAX)
    {
        DijkstraData data[this->G_MAX];
        dijkstra(this->edges, data, start);
        distance = (data[end].distance != DOUBLE_MAX?data[end].distance:-1);
        getShortPath(end, data, path);
        if(!path.empty() || distance != -1){
            path.push_back(end);
        }
    }
    return std::make_tuple(distance, path);
}

void Graph::dijkstra(EdgeNode **edges, DijkstraData *data, uint32_t start)
{
    if(edges == NULL || data == NULL)
        return;
    EdgeNode *cur_en;
    uint32_t neighbor;
    double weight;
    double dist_min;
    uint32_t n = start;
    //开始点
    data[start].distance = 0;
    while(data[n].visit == false) 
    {
        data[n].visit = true;
        cur_en = edges[n];

        while(cur_en != NULL) {
            neighbor = cur_en->key;
            weight = cur_en->weight;
            if((data[n].distance + weight) < data[neighbor].distance){
                data[neighbor].distance = data[n].distance + weight;
                data[neighbor].parent = n;
            }
            cur_en = cur_en->next;
        }
        //set the next current vertex to the vertex with the smallest distance
        dist_min = DOUBLE_MAX;
        for(uint32_t i = 0; i < this->G_MAX; i++){
            if(!data[i].visit && (data[i].distance < dist_min)){
                n = i;
                dist_min = data[i].distance;
            }
        }
    }
}

bool Graph::findNeighborsFromID(int id, std::vector<DNode> &nodes)
{
    bool ret = false;
    nodes.clear();
    if(id < this->G_MAX)
    {
        if(this->edges[id] != NULL)
        {
            DNode node;
            EdgeNode *curr = this->edges[id];
            while(curr != NULL)
            {
                node.start_id = id;
                node.end_id = curr->key;
                node.distance = curr->weight;
                nodes.push_back(node);
                curr = curr->next;
            }
            ret = true;
        }
    }
    return ret;
}

void Graph::print_Graph()
{
    for(int i = 0; i < this->G_MAX; i++)
    {
        if(this->edges[i] != NULL)
        {
            std::cout << "Vertex " << i << " has neighbors: " << std::endl;
            EdgeNode *curr = this->edges[i];
            while(curr != NULL){
                std::cout << curr->key << std::endl;
                curr = curr->next;
            }
        }
    }
}

}//namespace test
}//namespace cti
#endif
