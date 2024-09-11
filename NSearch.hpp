#pragma once

#ifndef _OPTIMIZATION_H_
#define _OPTIMIZATION_H_
#include <vector>

#include "node.hpp"


namespace LS {
	/// @brief 计算节点的邻域
	void neighbor(std::vector<Node*>& node, u32 size);

	/// @brief 点重定位操作
	void relocate(Solution& solution, u32& num, float size_near, bool& flag);

	/// @brief 2-opt操作
	void twoopt(Solution& solution, u32& num, bool& flag);

	/// @brief 两点交换操作
	void exchange(Solution& solution, u32& num, float size_near, bool& flag);

	/// @brief 点边交换操作
	void arcnode(Solution& solution, u32& num, float size_near, bool& flag);

	/// @brief 边交换操作
	void arcswap(Solution& solution, u32& num, bool& flag);

	/// @brief oropt 2 操作
	void oropt2(Solution& solution, u32& num, float size_near, bool& flag);

	/// @brief oropt 3 操作
	void oropt3(Solution& solution, u32& num, float size_near, bool& flag);

	/// @brief oropt 4 操作
	void oropt4(Solution& solution, u32& num, float size_near, bool& flag);

	/// @brief 交叉扰动优化操作
	void cross(Solution& solution, u32& num, bool& flag);
};  // namespace LS

namespace SHACK {
	void arcnode(Solution& solution, float threshold, u32 max_iter);
	void oropt(Solution& solution, float threshold, u32 max_iter);
	void arcswap(Solution& solution, float threshold, u32 max_iter);
	void optstar(Solution& solution, float threshold, u32 max_iter);
	void twoopt(Solution& solution, float threshold, u32 max_iter);
};  // namespace SHACK
#endif