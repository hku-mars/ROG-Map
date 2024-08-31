/*
Copyright (C) 2022 Hongkai Ye (kyle_yeh@163.com)
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/
#ifndef _NODE_H_
#define _NODE_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <utility>

using std::vector;

struct TreeNode
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	TreeNode() : parent(NULL), cost_from_start(DBL_MAX), cost_from_parent(0.0){};
	TreeNode *parent;
	Eigen::Vector3d x;
	double cost_from_start;
	double cost_from_parent;
	double heuristic_to_goal;
	double g_plus_h;
	std::list<TreeNode *> children;
};
typedef TreeNode *RRTNode3DPtr;
typedef vector<RRTNode3DPtr, Eigen::aligned_allocator<RRTNode3DPtr>> RRTNode3DPtrVector;
typedef vector<TreeNode, Eigen::aligned_allocator<TreeNode>> RRTNode3DVector;

class RRTNodeComparator
{
public:
	bool operator()(RRTNode3DPtr node1, RRTNode3DPtr node2)
	{
		return node1->g_plus_h > node2->g_plus_h;
	}
};

struct NodeWithStatus
{
	NodeWithStatus()
	{
		node_ptr = nullptr;
		is_checked = false;
		is_valid = false;
	};
	NodeWithStatus(const RRTNode3DPtr &n, bool checked, bool valid) : node_ptr(n), is_checked(checked), is_valid(valid){};
	RRTNode3DPtr node_ptr;
	bool is_checked;
	bool is_valid; // the segment from a center, not only the node
};

struct Neighbour
{
	Eigen::Vector3d center;
	vector<NodeWithStatus> nearing_nodes;
};

#endif
