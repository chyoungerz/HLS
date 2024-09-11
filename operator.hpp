#pragma once

#include <vector>
#ifndef _OPERATOR_HPP_
#define _OPERATOR_HPP_

#include "node.hpp"


namespace CHK {

	/// @brief 查找
	/// @param route
	/// @param seq(序号或指针)
	/// @return 位置
	u32 find(std::vector<Node*>& route, const u32 seq, u32 where);
	u32 find(std::vector<Node*>& route, Node* seq, u32 where);
}  // namespace CHK

/// @brief 扰动算子
/// @todo 破坏后的重建使用随机插入顺序，找到最好的插入顺序（重复大概50次），然后插入。
namespace PER {

	double cumlength(std::vector<Node*>& path);
	bool insert(Vehicle& vehicle, Node* node, u32 ctrl);
	bool insert(Solution& sol, Node* node, u32 ctrl);

	/// @brief 抛射链扰动
	/// @param sol 解
	/// @param k k条路径
	/// @param epoch 扰动最大失败次数
	void EjecChain(Solution& sol, u32 k, u32 epoch);

	/// @brief 破坏重建扰动
	/// @param sol 解
	/// @param k 破坏的邻域个数
	/// @param maxnode 最大节点数
	void RuinCreate(Solution& sol, u32 k, u32 maxnode);

	/// @brief 破坏重建优化扰动
	/// @param sol 解
	/// @param k 破坏的邻域个数
	/// @param maxnode 最大节点数
	/// @param epoch 最大停止次数
	void RuinCreate(Solution& sol, float k, std::vector<Node*>& maxnode, u32 epoch, u32 rule);
	void RuinCreate(Solution& sol, float k, std::vector<Node*>& maxnode, u32 epoch);

	/// @brief 抛射链扰动
	/// @param sol 解
	/// @param k k条路径
	/// @param epoch 扰动最大失败次数
	void EjecChain(Solution& sol, u32 k, u32 epoch, u32 rule);
}  // namespace PER

namespace OPS {
	bool remove(Solution& s, std::vector<Node*> r, const u32 k, double& saving);
	bool remove(Solution& s, std::vector<Node*> r, const u32 k, const u32 len, double& saving);
	bool insert(Solution& s, std::vector<Node*> r1, std::vector<Node*> r2, const u32 k, const u32 len, double& saving, bool& location);
	bool insert(Solution& s, std::vector<Node*> r, const u32 k, Node* node, double& saving, bool& location);
	bool reverse(Solution& s, Vehicle& r, const u32 f, const u32 t, u32 ctrl);
	bool swapmove(Solution& s, Vehicle& r1, Vehicle& r2, const u32 a, const u32 b, u32 ctrl);
	bool onepointmove(Solution& s, Vehicle& r1, Vehicle& r2, const u32 a, const u32 b, u32 ctrl);
	bool onepointmove(Solution& s, Vehicle& r, const u32 a, const u32 b, u32 ctrl);
	bool oropt(Solution& s, Vehicle& r1, Vehicle& r2, const u32 f, const u32 t, const u32 len, u32 ctrl);
	bool oropt(Solution& s, Vehicle& r, const u32 f, const u32 t, const u32 len, u32 ctrl);
	bool arcnode(Solution& s, Vehicle& r1, Vehicle& r2, const u32 a, const u32 b, u32 ctrl);
	bool arcswap(Solution& s, Vehicle& r1, Vehicle& r2, float c, const u32 a, const u32 b, u32 ctrl);
};  // namespace OPS
#endif