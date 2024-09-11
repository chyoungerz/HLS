#pragma once

#ifndef _SOLUTION_HPP
#define _SOLUTION_HPP
#include <vector>

#include "node.hpp"



// 基于贪婪策略的客户分配构造初始解
Solution greedynear(std::vector<const Node*>& nodes, const u32 depot_num, const u32 maxload);

// 基于贪婪策略的客户分配构造初始解
Solution nassign(std::vector<Node*> customers, std::vector<Node*> depots, const u32 maxload, const u32 routes, u32& ctrl);
Solution assign(std::vector<Node*> customers, std::vector<Node*> depots, const u32 maxload, const u32 routes, u32& ctrl);

#endif  // _SOLUTION_HPP