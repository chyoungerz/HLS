#include "NSearch.hpp"

#include <algorithm>
#include <random>
#include <unordered_set>
#include <utility>
#include <vector>

#include "algorithm.hpp"
#include "operator.hpp"

/**
 * @brief 这个函数在给定的解决方案上执行VNS重定位操作。
 *        它遍历解决方案中的每个路径，并尝试将一个节点从该路径重新定位到另一个路径。
 *        如果重定位成功，则更新解决方案并将标志参数设置为true。
 * @param s 要执行重定位的解决方案。
 * @param num 统计算子调用次数
 * @param flag 如果找到更好的解决方案，则设置为true的布尔标志。
 */
void LS::relocate(Solution& s, u32& num, float size, bool& flag) {
	Node* near{};    // 指向最近节点的指针,存储路径中的节点
	u32 tr{}, tp{}, seq{};  // to_route;to_path;from_route;from_path
	std::unordered_set<u32> tabu;
	tabu.reserve(32);
	flag = 0;
	bool skip{1};
// double saving{};                                        // 移动操作的节约值
// bool location{};                                        // 一个布尔标志，用于指示是否进行了任何移动
#ifndef NDEBUG
	s.debug_hash(1);
#endif
	for (u32 fr{0}, t = s.solution.size(); fr < t; fr++) {  // 遍历所有路径
		if (s.solution[fr].path.size() <= 2) continue;      // 如果路径长度小于等于2，则跳过
		if (!s.valid && s.solution[fr].valid()) continue;
		u32 fp{1};
		while (fp + 1 < s.solution[fr].path.size()) {  // 遍历路径中的节点
			if (tabu.find(s.solution[fr].path[fp]->seq) != tabu.end()) {
				fp++;
				continue;
			}
			seq = s.solution[fr].path[fp]->seq;
			for (u32 i{0}, m = s.solution[fr].path[fp]->distsort.size() * size; i < m; i++) {  // 遍历节点的邻居节点
				near = s.solution[fr].path[fp]->distsort[i].toNode;                           // 获取邻居节点
				tr = s.shash[near->seq];                                                      // 获取邻居节点所在的路径
				if (!s.valid && !s.solution[tr].valid()) continue;
				tp = CHK::find(s.solution[tr].path, near, 1);                                 // 获取邻居节点在其路径中的位置
				if (tr == fr) {
					tabu.emplace(seq);
					if (OPS::onepointmove(s, s.solution[fr], fp, tp, 0)) {  // 如果移动成功
#ifndef NDEBUG
						s.debug_hash(11);
#endif
						flag = 1;                                                                       // 设置标志为true
						num++;
						skip = 0;
						break;  // 跳出循环
					}
				} else {
					if (OPS::onepointmove(s, s.solution[fr], s.solution[tr], fp, tp, 0)) {              // 如果移动成功
						s.shash[seq] = tr;                                                              // 更新节点所在的路径
						flag = 1;                                                                       // 设置标志为true
						num++;
						skip = 0;
#ifndef NDEBUG
						s.debug_hash(12);
#endif
						break;  // 跳出循环
					}
				}
			}
			if (!s.valid || skip) fp++;
			skip = 1;
		}
		tabu.clear();
	}
	// s.evaluate();  // 移除空路径
	s.update();
}

/**
 * @brief 这个函数在给定的解决方案上执行2-opt邻域搜索。
 *        遍历解决方案中的所有路径，并尝试在每个路径上应用2-opt移动。
 *        如果找到更好的解决方案，则将标志设置为true。
 * @param s 要执行2-opt邻域搜索的解决方案。
 * @param num 统计算子调用次数
 * @param flag 如果找到更好的解决方案，则设置为true的布尔标志。
 */
void LS::twoopt(Solution& s, u32& num, bool& flag) {
	// double saving{};
	flag = 0;
	if (!s.valid) return;
	for (auto& r : s.solution) {
		if (r.path.size() < 4) continue;
		if (r.length > r.Limit) continue;
		for (u32 i{1}, n = r.path.size() - 1; i + 1 < n; i++) {
			for (u32 j{i + 1}; j < n; j++) {
				if (OPS::reverse(s, r, i, j, 0)) {
					flag = 1;
					num++;
				}
			}
		}
	}
	// s.evaluate();
	s.update();
}

/**
 * @brief 通过在路径之间交换节点来执行变邻域搜索。
 * @param s 要修改的解决方案。
 * @param num 统计算子调用次数
 * @param flag 一个布尔标志，用于指示是否进行了任何交换。
 */
void LS::exchange(Solution& s, u32& num, float size, bool& flag) {
	Node* near{};                                           // 指向最近节点的指针
	u32 tr{}, tp{}, fp_seq{}, ctrl{};                       // to_route;to_path;from_route;from_path
	// double saving{};                                        // 交换操作的节约值
	// if (!s.valid) ctrl = 1;
	flag = 0;
#ifndef NDEBUG
	s.debug_hash(7);
#endif
	for (u32 fr{0}, t = s.solution.size(); fr < t; fr++) {  // 遍历所有路径
		if (s.solution[fr].path.size() <= 2) continue;      // 如果路径长度小于等于2，则跳过
		u32 fp{1};
		while (fp + 1 < s.solution[fr].path.size()) {  // 遍历路径中的节点
			fp_seq = s.solution[fr].path[fp]->seq;
			for (u32 i{0}, m = s.solution[fr].path[fp]->distsort.size() * size; i < m; i++) {  // 遍历节点的邻居节点
				near = s.solution[fr].path[fp]->distsort[i].toNode;                           // 获取邻居节点
				tr = s.shash[near->seq];                                                      // 获取邻居节点所在路径的编号
				if (!s.solution[tr].valid() && !s.solution[fr].valid()) continue;
				tp = CHK::find(s.solution[tr].path, near, 2);                                 // 获取邻居节点在其所在路径中的位置
				if (OPS::swapmove(s, s.solution[fr], s.solution[tr], fp, tp, ctrl)) {         // 执行交换操作
					s.shash[fp_seq] = tr;                                                     // 更新路径哈希表
					s.shash[near->seq] = fr;                                                  // 更新路径哈希表
					flag = 1;                                                                 // 标记已进行交换操作
					num++;
#ifndef NDEBUG
					s.debug_hash(70);
#endif
					break;  // 跳出循环
				}
			}
			fp++;
		}
	}
	// s.evaluate();
	s.update();
}

/**
 * @brief 此函数对给定的解执行 Or-opt 2 邻域搜索。
 *        它接受 Solution 对象和标志作为输入，并在找到更好的解时修改 Solution 对象。
 *        如果找到更好的解，则将标志设置为 true，否则保持为 false。
 * @param s 要在其上执行邻域搜索的 Solution 对象。
 * @param num 统计算子调用次数
 * @param flag 一个布尔标志，用于指示是否找到更好的解。
 */
void LS::oropt2(Solution& s, u32& num, float size_near, bool& flag) {
	// 定义指向节点的指针 p1 和 p2，以及一个无序集合 tabu，用于存储禁忌表。
	Node *p1{}, *p2{};
	flag = 0;
	bool skip{1};
	std::unordered_set<u32> tabu;
	tabu.reserve(32);
	//  定义向量 p 和 neighbors，以及变量 tr、tp、size、fp 和 saving。
	std::vector<Node*> p, neighbors;
	u32 tr{}, tp{}, size{}, fp{};  // to_route;to_path;from_route;from_path
// double saving{};
// bool location{};
//  遍历解中的所有路径。
#ifndef NDEBUG
	s.debug_hash(2);
#endif
	for (u32 fr{0}, t = s.solution.size(); fr < t; fr++) {
		// 如果路径长度小于等于 4，则跳过该路径。
		if (s.solution[fr].path.size() <= 4) continue;
		if (!s.valid && s.solution[fr].valid()) continue;
		// 初始化 index 为 1，size 为 p1 的仓库排序列表的 20%。
		u32 index{1};
		size = s.solution[fr].path[index]->distsort.size() * size_near;
		// 当 index 小于路径长度减 2 时，执行以下循环。
		while (index + 2 < s.solution[fr].path.size()) {
			// 选择相邻的节点 p1 和 p2。
			p1 = s.solution[fr].path[index];
			p2 = s.solution[fr].path[index + 1];
			// 如果 (p1->seq << 10) + p2->seq 在禁忌表中，则跳过该节点。
			if (tabu.find((p1->seq << 10) + p2->seq) != tabu.end()) {
				index += 2;
				continue;
			}
			// 生成邻居节点。
			neighbors.emplace_back(p1);
			neighbors.emplace_back(p2);
			LS::neighbor(neighbors, size);
			fp = index;
			// 遍历邻居节点。
			for (u32 i{0}, m = neighbors.size(); i < m; i++) {
				// 获取邻居节点的路径和位置。
				tr = s.shash[neighbors[i]->seq];
				if (!s.valid && !s.solution[tr].valid()) continue;
				tp = CHK::find(s.solution[tr].path, neighbors[i], 3);
				// 如果邻居节点在同一路径上，则执行 Or-opt 2 操作。
				if (tr == fr) {
					if (OPS::oropt(s, s.solution[fr], fp, tp, 2, 0)) {
						flag = 1;
						num++;
						skip = 0;
						// 如果 fp 小于 tp，则将 (p1->seq << 10) + p2->seq 加入禁忌表。
						if (fp < tp) {
							tabu.emplace((p1->seq << 10) + p2->seq);
						}
#ifndef NDEBUG
						s.debug_hash(21);
#endif
						break;
					}
				} else {
					// 如果邻居节点在不同路径上，则执行  Or-opt 2 操作。
					if (OPS::oropt(s, s.solution[fr], s.solution[tr], fp, tp, 2, 0)) {
						flag = 1;
						num++;
						skip = 0;
						// 更新 s.shash。
						s.shash[p1->seq] = tr;
						s.shash[p2->seq] = tr;
#ifndef NDEBUG
						s.debug_hash(22);
#endif
						break;
					}
				}
			}
			neighbors.clear();
			if (!s.valid || skip) index++;
			skip = 1;
		}
		// 清空禁忌表。
		tabu.clear();
	}
	// s.evaluate();  // 移除空路径
	s.update();
}

/**
 * @brief 此函数对给定的解执行 Arc Node 邻域搜索操作。
 *        它遍历解中的所有路径，并为每个路径选择一对相邻节点，
 *        然后通过从第二个节点的仓库排序列表中删除一部分节点来生成一组邻居。
 *        然后尝试将每个邻居插入到所有其他路径中，并检查解是否改善。
 *        如果解得到改善，则更新解并继续下一对节点。
 *        如果解没有改善，则继续下一对节点。
 *        如果已经尝试过一对节点，则跳过该节点。
 * @param s 要在其上执行 Arc Node 邻域搜索操作的解。
 * @param num 统计算子调用次数
 * @param flag 一个布尔标志，如果操作改善了解，则设置为 true。
 */
void LS::arcnode(Solution& s, u32& num, float size_near, bool& flag) {
	// 定义指向节点的指针
	Node *p1{}, *p2{};
	flag = 0;
	bool skip{1};
	// 定义一个哈希表，用于存储禁忌表
	std::unordered_set<u32> tabu;
	tabu.reserve(32);
	// 定义两个节点的向量
	std::vector<Node*> p, neighbors;
	// 定义一些整数变量
	u32 tr{}, tp{}, size{}, fp{};  // to_route;to_path;from_route;from_path
	// 定义一个双精度浮点数变量
	double saving{};
	// 遍历解决方案中的每个路径
#ifndef NDEBUG
	s.debug_hash(3);
#endif
	for (u32 fr{0}, t = s.solution.size(); fr < t; fr++) {
		// 如果路径中的节点数小于等于4，则跳过该路径
		if (s.solution[fr].path.size() <= 4) continue;
		if (!s.valid && s.solution[fr].valid()) continue;
		// 初始化索引和邻域大小
		u32 index{1};
		size = s.solution[fr].path[index]->distsort.size() * size_near;
		// 遍历路径中的每个节点对
		while (index + 2 < s.solution[fr].path.size()) {
			// 获取当前节点对
			p1 = s.solution[fr].path[index];
			p2 = s.solution[fr].path[index + 1];
			// 如果当前节点对在禁忌表中，则跳过该节点对
			if (tabu.find((p1->seq << 10) + p2->seq) != tabu.end()) {
				index++;
				continue;
			}
			// 获取当前节点对的邻域
			neighbors.emplace_back(p1);
			neighbors.emplace_back(p2);
			LS::neighbor(neighbors, size);
			fp = index;
			// 遍历邻域中的每个节点
			for (u32 i{0}, m = neighbors.size(); i < m; i++) {
				// 获取邻域中节点所在的路径和位置
				tr = s.shash[neighbors[i]->seq];
				if (!s.valid && !s.solution[tr].valid()) continue;
				tp = CHK::find(s.solution[tr].path, neighbors[i], 4);
				// tp_seq = s.solution[tr].path[tp]->seq;
				//  如果邻域中的节点与当前节点对在同一路径上
				if (tr == fr) {
					// 尝试在当前路径上执行 Arc Node 邻域操作
					if (OPS::arcnode(s, s.solution[fr], s.solution[fr], fp, tp, saving)) {
						flag = 1;
						num++;
						skip = 0;
						// 如果操作成功，则将当前节点对添加到禁忌表中
						if (fp < tp)
							tabu.emplace((p1->seq << 10) + p2->seq);
#ifndef NDEBUG
						s.debug_hash(31);
#endif
						break;
					}
				} else {
					// 尝试在不同路径上执行 Arc Node 邻域操作
					if (OPS::arcnode(s, s.solution[fr], s.solution[tr], fp, tp, saving)) {
						flag = 1;
						num++;
						skip = 0;
						// 如果操作成功，则更新节点所在的路径
						s.shash[p1->seq] = tr;
						s.shash[p2->seq] = tr;
						s.shash[neighbors[i]->seq] = fr;
#ifndef NDEBUG
						s.debug_hash(32);
#endif
						break;
					}
				}
			}
			if (!s.valid || skip) index++;
			skip = 1;
			neighbors.clear();
		}
		// 清空禁忌表
		tabu.clear();
	}
	// s.evaluate();  // 移除空路径
	s.update();
}

/**
 * @brief 此函数对给定的解执行 Or-opt 3 邻域搜索。
 *        它接受 Solution 对象和标志作为输入，并在找到更好的解时修改 Solution 对象。
 *        如果找到更好的解，则将标志设置为 true，否则保持为 false。
 * @param s 要在其上执行邻域搜索的 Solution 对象。
 * @param num 统计算子调用次数
 * @param flag 一个布尔标志，用于指示是否找到更好的解。
 */
void LS::oropt3(Solution& s, u32& num, float size_near, bool& flag) {
	// 定义三个节点指针和一个无序集合
	Node *p1{}, *p2{}, *p3;
	flag = 0;
	bool skip{1};
	std::unordered_set<u32> tabu;
	tabu.reserve(32);
	//  定义两个节点向量和一些整数变量
	std::vector<Node*> p, neighbors;
	u32 tr{}, tp{}, size{}, fp{};  // to_route;to_path;from_route;from_path
	                               // double saving{};
	                               // bool location{};
	                               //  遍历解中的所有路径
#ifndef NDEBUG
	s.debug_hash(4);
#endif
	for (u32 fr{0}, t = s.solution.size(); fr < t; fr++) {
		// 如果路径长度小于等于5，则跳过
		if (s.solution[fr].path.size() <= 5) continue;
		if (!s.valid && s.solution[fr].valid()) continue;
		// 初始化索引和邻域大小
		u32 index{1};
		size = s.solution[fr].path[index]->distsort.size() * size_near;
		// 遍历路径中的节点
		while (index + 3 < s.solution[fr].path.size()) {
			// 获取三个相邻节点
			p1 = s.solution[fr].path[index];
			p2 = s.solution[fr].path[index + 1];
			p3 = s.solution[fr].path[index + 2];
			// 如果这个节点组合已经在禁忌表中，则跳过
			if (tabu.find((p1->seq << 20) + (p2->seq << 10) + p3->seq) != tabu.end()) {
				index++;
				continue;
			}
			// 获取邻域
			neighbors.emplace_back(p1);
			neighbors.emplace_back(p2);
			neighbors.emplace_back(p3);
			LS::neighbor(neighbors, size);
			fp = index;
			// 遍历邻域中的节点
			for (u32 i{0}, m = neighbors.size(); i < m; i++) {
				// 获取邻域中节点所在的路径
				tr = s.shash[neighbors[i]->seq];
				if (!s.valid && !s.solution[tr].valid()) continue;
				// 获取邻域中节点在路径中的位置
				tp = CHK::find(s.solution[tr].path, neighbors[i], 5);
				// 如果邻域中的节点在同一路径中
				if (tr == fr) {
					// 执行 Or-opt 3 操作
					if (OPS::oropt(s, s.solution[fr], fp, tp, 3, 0)) {
						flag = 1;
						num++;
						skip = 0;
						// 将这个节点组合加入禁忌表
						if (fp < tp)
							tabu.emplace((p1->seq << 20) + (p2->seq << 10) + p3->seq);
#ifndef NDEBUG
						s.debug_hash(41);
#endif
						break;
					}
				} else {
					// 如果邻域中的节点在不同路径中
					if (OPS::oropt(s, s.solution[fr], s.solution[tr], fp, tp, 3, 0)) {
						flag = 1;
						num++;
						skip = 0;
						// 更新节点所在路径的哈希表
						s.shash[p1->seq] = tr;
						s.shash[p2->seq] = tr;
						s.shash[p3->seq] = tr;
#ifndef NDEBUG
						s.debug_hash(42);
#endif
						break;
					}
				}
			}
			if (!s.valid || skip) index++;
			skip = 1;
			neighbors.clear();
		}
		// 清空禁忌表
		tabu.clear();
	}
	// s.evaluate();  // 移除空路径
	s.update();
}

/**
 * @brief 此函数对给定的解执行 Or-opt 4 邻域搜索。
 *        它接受 Solution 对象和标志作为输入，并在找到更好的解时修改 Solution 对象。
 *        如果找到更好的解，则将标志设置为 true，否则保持为 false。
 * @param s 要在其上执行邻域搜索的 Solution 对象。
 * @param num 统计算子调用次数
 * @param flag 一个布尔标志，用于指示是否找到更好的解。
 */
void LS::oropt4(Solution& s, u32& num, float size_near, bool& flag) {
	Node *p1{}, *p2{}, *p3{}, *p4{};  // 定义四个节点指针
	flag = 0;
	bool skip{1};
	std::unordered_set<u32> tabu;     // 定义一个无序集合，用于存储禁忌表
	tabu.reserve(32);                 // 预留32个空间
	std::vector<Node*> p, neighbors;  // 定义两个节点指针的向量
	u32 tr{}, tp{}, size{}, fp{};  // to_route;to_path;from_route;from_path
// double saving{};               // 定义一个double类型的变量，用于存储节省的距离
// bool location{};               // 定义一个bool类型的变量，用于存储位置信息
#ifndef NDEBUG
	s.debug_hash(5);
#endif
	for (u32 fr{0}, t = s.solution.size(); fr < t; fr++) {          // 遍历解中的所有路径
		if (s.solution[fr].path.size() <= 6) continue;              // 如果路径长度小于等于6，则跳过
		if (!s.valid && s.solution[fr].valid()) continue;
		u32 index{1};                                               // 定义一个索引变量
		size = s.solution[fr].path[index]->distsort.size() * size_near;  // 计算邻域大小
		while (index + 4 < s.solution[fr].path.size()) {                 // 遍历路径中的节点
			p1 = s.solution[fr].path[index];                        // 获取第一个节点
			p2 = s.solution[fr].path[index + 1];                    // 获取第二个节点
			p3 = s.solution[fr].path[index + 2];                    // 获取第三个节点
			p4 = s.solution[fr].path[index + 3];                    // 获取第四个节点
			// 如果这个节点已经在禁忌表中，则跳过
			if (tabu.find((p1->seq << 20) + (p2->seq << 10)) != tabu.end() && tabu.find((p3->seq << 10) + p4->seq) != tabu.end()) {
				index += 4;
				continue;
			}
			neighbors.emplace_back(p1);
			neighbors.emplace_back(p2);
			neighbors.emplace_back(p3);
			neighbors.emplace_back(p4);
			LS::neighbor(neighbors, size);  // 计算邻域
			fp = index;
			for (u32 i{0}, m = neighbors.size(); i < m; i++) {                      // 遍历邻域中的节点
				tr = s.shash[neighbors[i]->seq];                                    // 获取邻域中节点所在的路径
				if (!s.valid && !s.solution[tr].valid()) continue;
				tp = CHK::find(s.solution[tr].path, neighbors[i], 6);               // 获取邻域中节点在路径中的位置
				if (tr == fr) {                                                     // 如果邻域中的节点和当前路径中的节点在同一路径上
					if (OPS::oropt(s, s.solution[fr], fp, tp, 4, 0)) {              // 执行or-opt操作
						flag = 1;                                                   // 标记找到更好的解
						num++;
						skip = 0;
						if (fp < tp)
							tabu.emplace((p1->seq << 20) + (p2->seq << 10)), tabu.emplace((p3->seq << 10) + p4->seq);  // 将节点加入禁忌表
#ifndef NDEBUG
						s.debug_hash(51);
#endif
						break;
					}

				} else {                                                                            // 如果邻域中的节点和当前路径中的节点不在同一路径上
					if (OPS::oropt(s, s.solution[fr], s.solution[tr], fp, tp, 4, 0)) {              // 执行or-opt操作
						flag = 1;                                                                   // 标记找到更好的解
						num++;
						skip = 0;
						s.shash[p1->seq] = tr;  // 更新节点所在路径的哈希表
						s.shash[p2->seq] = tr;
						s.shash[p3->seq] = tr;
						s.shash[p4->seq] = tr;
#ifndef NDEBUG
						s.debug_hash(52);
#endif
						break;
					}
				}
			}
			if (!s.valid || skip) index++;  // 更新索引
			skip = 1;
			neighbors.clear();
		}
		tabu.clear();  // 清空禁忌表
	}
	// s.evaluate();  // 移除空路径
	s.update();
}

void LS::arcswap(Solution& s, u32& num, bool& flag) {
	flag = 0;
	float c = 1.0;  // 温度
	for (auto r1{s.solution.begin()}; r1 != s.solution.end(); ++r1) {
		if ((*r1).path.size() < 4) continue;
		for (auto r2 = r1; r2 != s.solution.end(); ++r2) {
			if (!s.valid && ((*r1).valid() && (*r2).valid())) continue;
			if ((*r1).seq != (*r2).seq) {  // 不同路径
				if ((*r2).path.size() < 4) continue;
				for (u32 i{1}, n = (*r1).path.size() - 2; i < n; i++) {
					for (u32 j{1}, m = (*r2).path.size() - 2; j < m; j++) {
						if (OPS::arcswap(s, *r1, *r2, c, i, j, 0)) {
							s.shash[(*r1).path[i]->seq] = (*r1).seq;
							s.shash[(*r1).path[i + 1]->seq] = (*r1).seq;
							s.shash[(*r2).path[j]->seq] = (*r2).seq;
							s.shash[(*r2).path[j + 1]->seq] = (*r2).seq;
							flag = 1;
							num++;
#ifndef NDEBUG
							s.debug_hash(53);
#endif
						}
					}
				}
			} else {
				if ((*r2).path.size() < 6) continue;
				for (u32 i{1}, n = (*r1).path.size() - 4; i < n; i++) {
					for (u32 j{2 + i}, m = (*r2).path.size() - 2; j < m; j++) {
						if (OPS::arcswap(s, *r1, *r2, c, i, j, 0)) {
							flag = 1;
							num++;
#ifndef NDEBUG
							s.debug_hash(54);
#endif
						}
					}
				}
			}
		}
	}
	// s.evaluate();
	s.update();
}

/// @brief 计算节点的邻域
/// @param node  输入节点
/// @param size 大小参数
/// @param neighbor  存储邻域的节点
void LS::neighbor(std::vector<Node*>& node, u32 size) {
	std::unordered_set<Node*> neighbor_set;
	neighbor_set.reserve((size + 1) << 2);
	if (node.size() == 2) {
		Node *n1{node[0]}, *n2{node[1]};
		// 将节点添加到邻居集合中
		for (u32 i{0}; i <= size; i++) {
			neighbor_set.emplace(n1->distsort[i].toNode);
			neighbor_set.emplace(n2->distsort[i].toNode);
		}
		// 从邻居集合中移除 n1 和 n2
		neighbor_set.erase(n1), neighbor_set.erase(n2);
		// 将邻居集合分配给邻居向量
		node.assign(neighbor_set.begin(), neighbor_set.end());
		if (node.size() > size) {
			// 根据距离对邻居向量进行部分排序
			std::partial_sort(node.begin(), node.begin() + size - 1, node.end(), [n1, n2](Node* a, Node* b) {
				return a->dists[n1->seq].dist + a->dists[n2->seq].dist < b->dists[n1->seq].dist + b->dists[n2->seq].dist;
			});
			// 调整邻居向量大小为 size
			node.resize(size);
		}
		return;
	}
	if (node.size() == 3) {
		Node *n1{node[0]}, *n2{node[1]}, *n3{node[2]};
		// 将节点添加到邻居集合中
		for (u32 i{0}; i <= size; i++) {
			neighbor_set.emplace(n1->distsort[i].toNode);
			neighbor_set.emplace(n2->distsort[i].toNode);
			neighbor_set.emplace(n3->distsort[i].toNode);
		}
		// 从邻居集合中移除 n1、n2 和 n3
		neighbor_set.erase(n1), neighbor_set.erase(n2), neighbor_set.erase(n3);
		// 将邻居集合分配给邻居向量
		node.assign(neighbor_set.begin(), neighbor_set.end());
		if (node.size() > size) {
			// 根据距离对邻居向量进行部分排序
			std::partial_sort(node.begin(), node.begin() + size - 1, node.end(), [n1, n2, n3](Node* a, Node* b) {
				return a->dists[n1->seq].dist + a->dists[n2->seq].dist + a->dists[n3->seq].dist < b->dists[n1->seq].dist + b->dists[n2->seq].dist + b->dists[n3->seq].dist;
			});
			// 调整邻居向量大小为 size
			node.resize(size);
		}
		return;
	}
	if (node.size() == 4) {
		Node *n1{node[0]}, *n2{node[1]}, *n3{node[2]}, *n4{node[3]};
		// 将节点添加到邻居集合中
		for (u32 i{0}; i <= size; i++) {
			neighbor_set.emplace(n1->distsort[i].toNode), neighbor_set.emplace(n2->distsort[i].toNode);
			neighbor_set.emplace(n3->distsort[i].toNode), neighbor_set.emplace(n4->distsort[i].toNode);
		}
		// 从邻居集合中移除 n1、n2、n3 和 n4
		neighbor_set.erase(n1), neighbor_set.erase(n2), neighbor_set.erase(n3), neighbor_set.erase(n4);
		// 将邻居集合分配给邻居向量
		node.assign(neighbor_set.begin(), neighbor_set.end());
		// 根据距离对邻居向量进行部分排序
		if (node.size() > size) {
			std::partial_sort(node.begin(), node.begin() + size - 1, node.end(), [n1, n2, n3, n4](Node* a, Node* b) {
				return a->dists[n1->seq].dist + a->dists[n2->seq].dist + a->dists[n3->seq].dist + a->dists[n4->seq].dist < b->dists[n1->seq].dist + b->dists[n2->seq].dist + b->dists[n3->seq].dist + b->dists[n4->seq].dist;
			});
			// 调整邻居向量大小为 size
			node.resize(size);
		}
		return;
	}
}

void SHACK::arcnode(Solution& s, float threshold, u32 max_iter) {
	u32 size_s = s.solution.size();
	std::random_device rd;
	Xoshiro::Xoshiro128ss gen(rd());
	std::uniform_int_distribution<> dis_s(0, size_s - 1);
	u32 p1{}, p2{}, r1{}, r2{};
	Node *n1{}, *n2{}, *n3{};
	double dif_obj{}, new_tardiness{}, new_length_1{}, new_length_2{};
	int m = 100;
	while (m--) {
		r1 = dis_s(gen), r2 = dis_s(gen);
		if (m-- < 0) return;
		if (r1 == r2) continue;
		if (s.solution[r1].path.size() < 4 || s.solution[r2].path.size() < 3) continue;
		break;
	}
	m = 100;
	u32 iter{};
	while (iter++ < max_iter) {
		p1 = gen() % (s.solution[r1].path.size() - 3) + 1;
		p2 = gen() % (s.solution[r2].path.size() - 2) + 1;
		int dif_load = s.solution[r1].path[p1]->demand + s.solution[r1].path[p1 + 1]->demand - s.solution[r2].path[p2]->demand;
		if (dif_load + s.solution[r2].load > s.solution[r2].capacity || s.solution[r1].load - dif_load > s.solution[r1].capacity) continue;
		n1 = s.solution[r1].path[p1], n2 = s.solution[r1].path[p1 + 1], n3 = s.solution[r2].path[p2];
		s.solution[r1].path[p1] = n3, s.solution[r2].path[p2] = n1;
		s.solution[r1].path.erase(s.solution[r1].path.begin() + p1 + 1);
		s.solution[r2].path.emplace(s.solution[r2].path.begin() + p2 + 1, n2);
		new_tardiness = priority(s);
		new_length_1 = s.solution[r1].path_cumlength();
		new_length_2 = s.solution[r2].path_cumlength();
		dif_obj = v_aim(new_length_1 + new_length_2 - s.solution[r1].cumlength - s.solution[r2].cumlength, new_tardiness - s.alltardiness);
		if (dif_obj / s.allobj <= threshold) {
			s.alltardiness = new_tardiness;
			s.solution[r1].cumlength = new_length_1;
			s.solution[r2].cumlength = new_length_2;
			s.solution[r1].load -= dif_load;
			s.solution[r2].load += dif_load;
			s.shash[n1->seq] = r2;
			s.shash[n2->seq] = r2;
			s.shash[n3->seq] = r1;
			s.update();
			return;
		} else {
			s.solution[r1].path[p1] = n1, s.solution[r2].path[p2] = n3;
			s.solution[r2].path.erase(s.solution[r2].path.begin() + p2 + 1);
			s.solution[r1].path.emplace(s.solution[r1].path.begin() + p1 + 1, n2);
		}
	}
}

void SHACK::arcswap(Solution& s, float threshold, u32 max_iter) {
	u32 size_s = s.solution.size();
	std::random_device rd;
	Xoshiro::Xoshiro128ss gen(rd());
	std::uniform_int_distribution<> dis_s(0, size_s - 1);
	u32 p1{}, p2{}, r1{}, r2{};
	double dif_obj{}, new_tardiness{}, new_length_1{}, new_length_2{};
	int m = 100;
	while (m--) {
		r1 = dis_s(gen), r2 = dis_s(gen);
		if (m-- < 0) return;
		if (r1 == r2) continue;
		if (s.solution[r1].path.size() < 4 || s.solution[r2].path.size() < 4) continue;
		break;
	}
	m = 100;
	u32 iter{};
	while (iter++ < max_iter) {
		p1 = gen() % (s.solution[r1].path.size() - 3) + 1;
		p2 = gen() % (s.solution[r2].path.size() - 3) + 1;
		int dif_load = s.solution[r1].path[p1]->demand + s.solution[r1].path[p1 + 1]->demand - s.solution[r2].path[p2]->demand - s.solution[r2].path[p2 + 1]->demand;
		if (dif_load + s.solution[r2].load > s.solution[r2].capacity || s.solution[r1].load - dif_load > s.solution[r1].capacity) continue;
		std::swap(s.solution[r1].path[p1], s.solution[r2].path[p2]);
		std::swap(s.solution[r1].path[p1 + 1], s.solution[r2].path[p2 + 1]);
		new_tardiness = priority(s);
		new_length_1 = s.solution[r1].path_cumlength();
		new_length_2 = s.solution[r2].path_cumlength();
		dif_obj = v_aim(new_length_1 + new_length_2 - s.solution[r1].cumlength - s.solution[r2].cumlength, new_tardiness - s.alltardiness);
		if (dif_obj / s.allobj <= threshold) {
			s.alltardiness = new_tardiness;
			s.solution[r1].cumlength = new_length_1;
			s.solution[r2].cumlength = new_length_2;
			s.solution[r1].load -= dif_load;
			s.solution[r2].load += dif_load;
			s.shash[s.solution[r1].path[p1]->seq] = r1;
			s.shash[s.solution[r1].path[p1 + 1]->seq] = r1;
			s.shash[s.solution[r2].path[p2]->seq] = r2;
			s.shash[s.solution[r2].path[p2 + 1]->seq] = r2;
			s.update();
			return;
		} else {
			std::swap(s.solution[r1].path[p1], s.solution[r2].path[p2]);
			std::swap(s.solution[r1].path[p1 + 1], s.solution[r2].path[p2 + 1]);
		}
	}
}

void SHACK::oropt(Solution& s, float threshold, u32 max_iter) {
	u32 size_s = s.solution.size();
	std::random_device rd;
	Xoshiro::Xoshiro128ss gen(rd());
	std::uniform_int_distribution<> dis_s(0, size_s - 1);
	u32 p1{}, p2{}, r1{}, r2{};
	Node *n1{}, *n2{};
	double dif_obj{}, new_tardiness{}, new_length_1{}, new_length_2{};
	int m = 100;
	while (m--) {
		r1 = dis_s(gen), r2 = dis_s(gen);
		if (m-- < 0) return;
		if (r1 == r2) continue;
		if (s.solution[r1].path.size() < 4) continue;
		break;
	}
	m = 100;
	u32 iter{};
	while (iter++ < max_iter) {
		p1 = gen() % (s.solution[r1].path.size() - 3) + 1;
		p2 = gen() % (s.solution[r2].path.size() - 2) + 1;
		int dif_load = s.solution[r1].path[p1]->demand + s.solution[r1].path[p1 + 1]->demand;
		if (dif_load + s.solution[r2].load > s.solution[r2].capacity) continue;
		n1 = s.solution[r1].path[p1], n2 = s.solution[r1].path[p1 + 1];
		s.solution[r2].path.insert(s.solution[r2].path.begin() + p2, {n1, n2});
		s.solution[r1].path.erase(s.solution[r1].path.begin() + p1, s.solution[r1].path.begin() + p1 + 2);
		new_tardiness = priority(s);
		new_length_1 = s.solution[r1].path_cumlength();
		new_length_2 = s.solution[r2].path_cumlength();
		dif_obj = v_aim(new_length_1 + new_length_2 - s.solution[r1].cumlength - s.solution[r2].cumlength, new_tardiness - s.alltardiness);
		if (dif_obj / s.allobj <= threshold) {
			s.alltardiness = new_tardiness;
			s.solution[r1].cumlength = new_length_1;
			s.solution[r2].cumlength = new_length_2;
			s.solution[r1].load -= dif_load;
			s.solution[r2].load += dif_load;
			s.shash[n1->seq] = r2;
			s.shash[n2->seq] = r2;
			s.update();
			return;
		} else {
			s.solution[r1].path.insert(s.solution[r1].path.begin() + p1, {n1, n2});
			s.solution[r2].path.erase(s.solution[r2].path.begin() + p2, s.solution[r2].path.begin() + p2 + 2);
		}
	}
}

void SHACK::twoopt(Solution& s, float threshold, u32 max_iter) {
	u32 size_s = s.solution.size();
	u32 max_path{};
	int m{};
	for (auto& r : s.solution) {
		if (r.path.size() > max_path) max_path = r.path.size();
	}
	if (max_path < 5) return;
	std::random_device rd;
	Xoshiro::Xoshiro128ss gen(rd());
	std::uniform_int_distribution<> dis_s(0, size_s - 1);
	int p1{}, p2{};
	u32 r{};
	double dif_obj{}, new_tardiness{}, new_length{};
	while (1) {
		r = dis_s(gen);
		if (s.solution[r].path.size() >= 5) break;
	}
	u32 iter{};
	while (iter++ < max_iter) {
		m = 100;
		while (1) {
			p1 = gen() % (s.solution[r].path.size() - 2) + 1;
			p2 = gen() % (s.solution[r].path.size() - 2) + 1;
			if (std::abs(p1 - p2) > 1) break;
			if (m-- < 0) return;
		}
		if (p1 > p2)
			std::swap(p1, p2);
		std::reverse(s.solution[r].path.begin() + p1, s.solution[r].path.begin() + p2 + 1);
		new_tardiness = priority(s);
		new_length = s.solution[r].path_cumlength();
		dif_obj = v_aim(new_length - s.solution[r].cumlength, new_tardiness - s.alltardiness);
		if (dif_obj / s.allobj <= threshold) {
			s.alltardiness = new_tardiness;
			s.solution[r].cumlength = new_length;
			s.update();
			return;
		} else {
			std::reverse(s.solution[r].path.begin() + p1, s.solution[r].path.begin() + p2 + 1);
		}
	}
}