#ifndef TREE_HPP_
#define TREE_HPP_

#include <cmath>
#include <vector>
#include <algorithm>
#include <functional>
#include <iostream>
#include <stack>
#include "visualization_msgs/msg/marker_array.hpp"

class PointXY
{
public:
  double x;
  double y;
  PointXY() : x(0), y(0) {} // Default constructor
  // initialize with x and y
  PointXY(double x_val, double y_val) : x(x_val), y(y_val) {}
}; 
// X, Y and orientation
class PointXYori : public PointXY
{
public:
  double ori;
  PointXYori() : ori(0) {} // Default constructor
  // initialize with x and y
  PointXYori(double x_val, double y_val, double ori_val) : PointXY(x_val, y_val), ori(ori_val) {}
};

class PointXYZI
{
public:
  double x;
  double y;
  double z;
  double intensity;
};

class TreeNode
{
public:
    PointXYori pos;
    float prob = 0.5;
    std::vector<TreeNode *> children;
    TreeNode(PointXYori pos) : pos(pos) {}
};

class Tree
{

public:
    // create a root node and reserve memory for it
    TreeNode *root = new TreeNode(PointXYori(0.0, 0.0, 0.0));

    TreeNode *getRoot()
    {
        return root;
    }

    TreeNode *modifyRoot(PointXYori pos)
    {
        root->pos = pos;
        return root;
    }

    TreeNode *modifyNode(TreeNode *node, PointXYori pos)
    {
        node->pos = pos;
        return node;
    }

    TreeNode *addNode(TreeNode *parent, PointXYori pos)
    {
        TreeNode *newNode = new TreeNode(pos);
        parent->children.push_back(newNode);
        return newNode;
    }
    // recursive function to visit all nodes in the tree
    void printTree(TreeNode *node, std::string indent = "")
    {
        if (node != nullptr)
        {
            if (root == node)
            {
                // skip printing root node
            }
            else
            {
                std::cout << indent << "(" << node->pos.x << ", " << node->pos.y << ")" << std::endl;
            }
            for (TreeNode *child : node->children)
            {
                TreeNode *father = node;
                std::cout << "[" << father->pos.x << ", " << father->pos.y << "] ";
                printTree(child, indent + "->");
            }
        }
    }
    // recursive function to calculate the size of the tree
    int getSize(TreeNode *node)
    {
        int size = 1; // start with size 1 for the current node

        if (node != nullptr)
        {
            for (TreeNode *child : node->children)
            {
                size += getSize(child); // recursively calculate the size of each child
            }
        }

        return size;
    }

    // a recursive function to return all the leaves of the tree as a node
    std::vector<TreeNode*> getLeaves(TreeNode* node)
    {
        std::vector<TreeNode*> leaves;

        if (node != nullptr)
        {
            if (node->children.empty())
            {
                leaves.push_back(node);
            }
            else
            {
                for (TreeNode* child : node->children)
                {
                    std::vector<TreeNode*> childLeaves = getLeaves(child);
                    leaves.insert(leaves.end(), childLeaves.begin(), childLeaves.end());
                }
            }
        }

        return leaves;
    }

    // alternative way to print (and walk thru) the whole tree (non-recursive)
    void printTreeAlt(TreeNode *root)
    {
        if (root == nullptr)
        {
            return;
        }

        std::stack<TreeNode *> stack;
        stack.push(root);

        while (!stack.empty())
        {
            TreeNode *node = stack.top();
            stack.pop();

            if (node != root)
            {
                std::cout << "(" << node->pos.x << ", " << node->pos.y << ")" << std::endl;
            }

            for (TreeNode *child : node->children)
            {
                std::cout << "[" << node->pos.x << ", " << node->pos.y << "] ";
                stack.push(child);
            }
        }
    }

    // recursive function to visualize the tree in rviz
    void markerTree(TreeNode *node, visualization_msgs::msg::Marker &line1_marker, std::string indent = "")
    {
        if (node != nullptr)
        {
            if (root == node)
            {
                // skip root node
            }
            else
            {
                geometry_msgs::msg::Point p_node;
                p_node.x = node->pos.x;
                p_node.y = node->pos.y;
                p_node.z = 0.0;
                line1_marker.points.push_back(p_node);
            }
            for (TreeNode *child : node->children)
            {
                TreeNode *father = node;
                geometry_msgs::msg::Point p_father;
                p_father.x = father->pos.x;
                p_father.y = father->pos.y;
                p_father.z = 0.0;
                line1_marker.points.push_back(p_father);
                markerTree(child, line1_marker, indent + "->");
            }
        }
    }
    // recursive function to visualize the nodes in rviz
    void markerNodeTree(TreeNode *node, visualization_msgs::msg::Marker &node_marker, std::string indent = "")
    {
        if (node != nullptr)
        {
            if (root == node)
            {
                // skip root node
            }
            else
            {
                geometry_msgs::msg::Point p_node;
                p_node.x = node->pos.x;
                p_node.y = node->pos.y;
                p_node.z = 0.0;
                node_marker.points.push_back(p_node);
            }
            for (TreeNode *child : node->children)
            {
                markerTree(child, node_marker, indent + "->");
            }
        }
    }


};

#endif // TREE_HPP_
