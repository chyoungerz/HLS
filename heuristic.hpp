#pragma once

#include <vector>
#ifndef _HEURISTICS_HPP_
#define _HEURISTICS_HPP_

#include "node.hpp"

// SA算法
class HLSVND {
  public:
	Solution bestSol;          // 最好解
	Solution sol;              // 当前解
	Solution initSol;          // 初始解
	std::vector<Node*> nodes;  // 全部节点
	std::vector<Node*> depots;
	std::vector<Node*> customers;
	std::vector<double> hists;
	Info info;
	u32 depotnum;  // 厂站数
	u32 vehicles;  // 最大车辆数
	u32 ctrl{0};   // 控制变量

	/// @brief
	void init(std::vector<Node*>& node, std::vector<Node*>& depot, std::vector<Node*>& customer, const u32 depot_num, u32 maxload, u32 routes);
	/// @todo 参数 迭代次数30-50次， 没有提升次数20，
	void run();
	/// @brief reset
	void reset();
};

class SA {
  public:
	Solution bestSol;          // 最好解
	Solution sol;              // 当前解
	Solution initSol;          // 初始解
	std::vector<Node*> nodes;  // 全部节点
	std::vector<Node*> depots;
	std::vector<Node*> customers;
	Info info;
	u32 depotnum;  // 厂站数
	u32 vehicles;  // 最大车辆数
	u32 ctrl{0};   // 控制变量

	/// @brief
	void init(std::vector<Node*>& node, std::vector<Node*>& depot, std::vector<Node*>& customer, const u32 depot_num, u32 maxload, u32 routes);
	/// @todo 参数 迭代次数30-50次， 没有提升次数20，
	void run();
	/// @brief reset
	void reset();
};

class VNS {
  public:
	Solution bestSol;          // 最好解
	Solution sol;              // 当前解
	Solution initSol;          // 初始解
	std::vector<Node*> nodes;  // 全部节点
	std::vector<Node*> depots;
	std::vector<Node*> customers;
	Info info;
	u32 depotnum;  // 厂站数
	u32 vehicles;  // 最大车辆数
	u32 ctrl{0};   // 控制变量

	/// @brief
	void init(std::vector<Node*>& node, std::vector<Node*>& depot, std::vector<Node*>& customer, const u32 depot_num, u32 maxload, u32 routes);
	/// @todo 参数 迭代次数30-50次， 没有提升次数20，
	void run();
	/// @brief reset
	void reset();
};

#endif