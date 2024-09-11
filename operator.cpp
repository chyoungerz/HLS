#include "operator.hpp"

#include <algorithm>
#include <numeric>
#include <random>
#include <vector>

#include "algorithm.hpp"


u32 CHK::find(std::vector<Node*>& route, const u32 seq, u32 where) {
	for (u32 i = route.size() - 1; i > 0; i--) {
		if (route[i]->seq == seq) return i;
	}
#ifndef NDEBUG
	std::cerr << "没有找到节点:" << seq << "\n";
	throw "没有找到节点";
#endif
	std::exit(-1);
}

u32 CHK::find(std::vector<Node*>& route, Node* seq, u32 where) {
	for (u32 i = route.size() - 1; i > 0; i--) {
		if (route[i] == seq) return i;
	}
#ifndef NDEBUG
	std::cerr << "没有找到节点:" << seq->seq << " " << where << "\n";
	throw "没有找到节点";
#endif
	std::exit(-1);
}

double PER::cumlength(std::vector<Node*>& path) {
	double cumlength{};
	for (u64 j{0}, n{path.size() - 2}; j < n; j++) {
		cumlength += (n - j) * path[j]->dists[path[j + 1]->seq].dist;
	}
	return cumlength;
}

bool PER::insert(Vehicle& vehicle, Node* node, u32 ctrl) {
	if (vehicle.load + node->demand > vehicle.capacity) return false;
	vehicle.load += node->demand;
	vehicle.path.emplace(vehicle.path.end() - 1, node);
	std::vector<Node*> path{vehicle.path};
	double minlength{1000000000.0};
	u32 size = path.size();
	if (path[size - 3]->end >= path[size - 2]->end)
		minlength = PER::cumlength(path);
	for (u32 i = path.size() - 3; i > 0; i--) {
		std::swap(path[i], path[i + 1]);
		if (path[i]->end >= path[i + 1]->end) {
			double tmp = PER::cumlength(path);
			if (tmp < minlength) {
				vehicle.path = path;
				minlength = tmp;
			}
		}
	}
	return true;
}

bool PER::insert(Solution& sol, Node* node, u32 ctrl) {
	u32 best_seq{}, seq{0};
	std::vector<Node*> best_path{};
	double minlength{1000000000.0};
	for (auto& ve : sol.solution) {
		seq++;
		if (ve.load + node->demand > ve.capacity && !(ctrl & FORCE)) continue;
		ve.path.emplace(ve.path.begin() + 1, node);
		u32 size = ve.path.size() - 1;
		double tmp{};
		if (size <= 2) {  // 空的
			tmp = PER::cumlength(ve.path) - ve.cumlength;
			if (tmp < minlength) {
				best_path = ve.path;
				best_seq = seq - 1;
				minlength = tmp;
			}
		} else {  // 非空
			if (ve.path[1]->end >= ve.path[2]->end) {
				tmp = PER::cumlength(ve.path) - ve.cumlength;
				if (tmp < minlength) {
					best_path = ve.path;
					best_seq = seq - 1;
					minlength = tmp;
				}
			}
			for (u32 i{2}; i < size; i++) {
				std::swap(ve.path[i], ve.path[i - 1]);
				if (ve.path[i - 1]->end >= ve.path[i]->end && ve.path[i]->end >= ve.path[i + 1]->end) {
					tmp = PER::cumlength(ve.path) - ve.cumlength;
					if (tmp < minlength) {
						best_path = ve.path;
						best_seq = seq - 1;
						minlength = tmp;
					}
				}
			}
		}
		ve.path.erase(ve.path.end() - 2);
	}
	if (best_path.empty()) return false;
	sol.solution[best_seq].path = best_path;
	sol.solution[best_seq].cumlength += minlength;
	sol.solution[best_seq].load += node->demand;
	return true;
}

void PER::EjecChain(Solution& sol, u32 k, u32 epoch) {
	u32 size_s = sol.solution.size();
	if (k >= size_s) k = size_s;
	if (k == 0 || k == 1) return;
	std::vector<u32> select_s(size_s, 0);
	std::iota(select_s.begin() + 1, select_s.end(), 1);
	std::random_device rd;
	Xoshiro::Xoshiro128ss gen(rd());
	std::shuffle(select_s.begin(), select_s.end(), gen);
	select_s.resize(k);
	select_s.emplace_back(select_s.front());
	for (u32 i{0}; i < k; i++) {
		u32 range_a = sol.solution[select_s[i]].path.size() - 2;
		u32 range_b = sol.solution[select_s[i + 1]].path.size() - 2;
		int n = epoch;
		while (n--) {
			u32 index_a{gen() % range_a + 1};
			u32 index_b{gen() % range_b + 1};
			int difload = sol.solution[select_s[i]].path[index_a]->demand - sol.solution[select_s[i + 1]].path[index_b]->demand;
			if (sol.solution[select_s[i]].load - difload > sol.solution[select_s[i]].capacity || sol.solution[select_s[i + 1]].load + difload > sol.solution[select_s[i + 1]].capacity)
				continue;
			sol.solution[select_s[i]].load -= difload;
			sol.solution[select_s[i + 1]].load += difload;
			sol.shash[sol.solution[select_s[i]].path[index_a]->seq] = sol.solution[select_s[i + 1]].seq;
			sol.shash[sol.solution[select_s[i + 1]].path[index_b]->seq] = sol.solution[select_s[i]].seq;
			std::swap(sol.solution[select_s[i + 1]].path[index_b], sol.solution[select_s[i]].path[index_a]);
			sol.solution[select_s[i + 1]].path_cumlength(1);
			sol.solution[select_s[i]].path_cumlength(1);
			sol.alltardiness = priority(sol);
			// break;
		}
	}
	sol.update();
}

void PER::RuinCreate(Solution& sol, float k, std::vector<Node*>& maxnode, u32 epoch) {
	// ruin
	std::random_device rd;
	Xoshiro::Xoshiro128ss gen(rd());
	u32 size = maxnode.size();
	std::uniform_int_distribution<> disceate(0, size - 1);
	std::vector<Node*> near;
	// double allength{sol.allength};
	if (sol.solution.size() > sol.maxvehicle) {  // 移除最短
		u32 reducevehicle = sol.solution.size() - sol.maxvehicle;
		std::sort(sol.solution.begin(), sol.solution.end(), [](Vehicle& a, Vehicle& b) -> bool { return a.path.size() > b.path.size(); });
		for (u32 i{0}; i < reducevehicle; i++) {
			near.insert(near.end(), sol.solution.back().path.begin() + 1, sol.solution.back().path.end() - 1);
			sol.solution.pop_back();
		}
		sol.update_hash();
	} else {
		u32 index = maxnode[disceate(gen)]->seq;
		u32 rnode = sol.shash[index];
		u32 locate{CHK::find(sol.solution[rnode].path, index, 10)};
		near.reserve(size * k + 1);
		for (u32 i{0}; i < k * size; i++) {
			near.emplace_back(sol.solution[rnode].path[locate]->distsort[i].toNode);
		}
		// 移除node
		for (u32 i{0}, n = near.size(); i < n; i++) {
			index = sol.shash[near[i]->seq];
			locate = CHK::find(sol.solution[index].path, near[i], 11);
			sol.solution[index].load -= sol.solution[index].path[locate]->demand;       // load
			sol.solution[index].path.erase(sol.solution[index].path.begin() + locate);  // path
			                                                                            // sol.shash.erase(n->seq);                                                    // hash
		}
		std::for_each(sol.solution.begin(), sol.solution.end(), [](Vehicle& v) { v.path_cumlength(1); });
		sol.update_hash();
		// std::sort(sol.solution.begin(), sol.solution.end(), [](Vehicle& a, Vehicle& b) -> bool { return a.path.size() > b.path.size(); });
	}
	// 插入
	Solution s{sol}, best_sol{sol};
	best_sol.valid = 0;
	bool error{0}, flag{0};
	double flength{1000000000.0};
	while (epoch) {
		std::shuffle(near.begin(), near.end(), gen);  // 打乱
		for (auto& n : near) {
			if (!PER::insert(s, n, 0)) {
				error = 1;
				break;
			}
		}
		epoch--;
		if (error) {  // 失败
			if (epoch > 0) {
				s = sol;
				error = 0;
				continue;
			}
			std::sort(near.begin(), near.end(), [](Node* a, Node* b) { return a->demand > b->demand; });  // 排序
			for (auto& n : near) {
				std::sort(sol.solution.begin(), sol.solution.end(), [](Vehicle& a, Vehicle& b) -> bool { return a.load > b.load; });
				std::vector<Node*> ve{sol.solution.back().path};
				sol.solution.back().load += n->demand;
				ve.emplace(ve.begin() + 1, n);
				u32 size = ve.size() - 1;
				double tmp{}, best{1000000000.0};
				if (ve[1]->end >= ve[2]->end) {
					tmp = PER::cumlength(ve);
					sol.solution.back().path = ve;
					sol.solution.back().cumlength = tmp;
					best = tmp;
				}
				for (u32 i{2}; i < size; i++) {
					std::swap(ve[i], ve[i - 1]);
					if (ve[i - 1]->end >= ve[i]->end && ve[i]->end >= ve[i + 1]->end) {
						tmp = PER::cumlength(ve);
						if (tmp < best) {
							sol.solution.back().path = ve;
							sol.solution.back().cumlength = tmp;
							best = tmp;
						}
					}
				}
			}
#ifdef PR
			sol.alltardiness = priority(sol);
#endif
			sol.update();
			flag = 0;
			break;
		}
#ifdef PR
		s.alltardiness = priority(s);
#endif
		s.update();
		if (s.allobj < flength) {
			flength = s.allobj;
			flag = 1;
			best_sol = s;
			best_sol.valid = 1;
		}
		s = sol;
		error = 0;
	}
	if (flag)
		sol = best_sol;
	sol.update_seq();
	sol.update_hash(1);
}

void PER::RuinCreate(Solution& sol, float k, std::vector<Node*>& maxnode, u32 epoch, u32 rule) {
	// ruin
	std::random_device rd;
	Xoshiro::Xoshiro128ss gen(rd());
	u32 size = maxnode.size();
	std::uniform_int_distribution<> disceate(0, size - 1);
	std::vector<Node*> near;
	// double allength{sol.allength};
	if (sol.solution.size() > sol.maxvehicle || rule & 1) {  // 移除最短
		u32 reducevehicle = sol.solution.size() - sol.maxvehicle;
		std::sort(sol.solution.begin(), sol.solution.end(), [](Vehicle& a, Vehicle& b) -> bool { return a.path.size() > b.path.size(); });
		for (u32 i{0}; i < reducevehicle; i++) {
			near.insert(near.end(), sol.solution.back().path.begin() + 1, sol.solution.back().path.end() - 1);
			sol.solution.pop_back();
		}
		sol.update_hash();
	} else {
		u32 index = maxnode[disceate(gen)]->seq;
		u32 rnode = sol.shash[index];
		u32 locate{CHK::find(sol.solution[rnode].path, index, 10)};
		near.reserve(size * k + 1);
		for (u32 i{0}; i < k * size; i++) {
			near.emplace_back(sol.solution[rnode].path[locate]->distsort[i].toNode);
		}
		// 移除node
		for (u32 i{0}, n = near.size(); i < n; i++) {
			index = sol.shash[near[i]->seq];
			locate = CHK::find(sol.solution[index].path, near[i], 11);
			sol.solution[index].load -= sol.solution[index].path[locate]->demand;       // load
			sol.solution[index].path.erase(sol.solution[index].path.begin() + locate);  // path
			                                                                            // sol.shash.erase(n->seq);                                                    // hash
		}
		std::for_each(sol.solution.begin(), sol.solution.end(), [](Vehicle& v) { v.path_cumlength(1); });
		sol.update_hash();
		// std::sort(sol.solution.begin(), sol.solution.end(), [](Vehicle& a, Vehicle& b) -> bool { return a.path.size() > b.path.size(); });
	}
	Solution s{sol}, best_sol{sol};
	best_sol.valid = 0;
	// 插入
	bool error{0}, flag{0};
	double flength{1000000000.0}, ilength{1000000000.0};
	while (epoch) {
		std::shuffle(near.begin(), near.end(), gen);  // 打乱
		if (rule & 1 << 1) {                          // 随机
			for (auto& n : near) {
				if (!PER::insert(s, n, 0)) {
					error = 1;
					PER::insert(s, n, 1);
				}
			}
		} else {  // 后悔
			      // todo
		}
		epoch--;
		if (error) {  // 失败
#ifdef PR
			s.alltardiness = priority(s);
#endif

			s.update();
			s.valid = false;
			if (best_sol.valid == 0 && s.allobj < ilength) {
				ilength = s.allobj;
				flag = 1;
				best_sol = s;
				best_sol.valid = 0;
			}
			s = sol;
			error = 0;
			continue;
		}
#ifdef PR
		s.alltardiness = priority(s);
#endif

		s.update();
		if (s.allobj < flength) {
			flength = s.allobj;
			flag = 1;
			best_sol = s;
			best_sol.valid = 1;
		}
		s = sol;
		error = 0;
	}
	if (flag)
		sol = best_sol;
	else {
		throw "per failed";
	}
	sol.update_seq();
	sol.update_hash(1);
}

void PER::EjecChain(Solution& sol, u32 k, u32 epoch, u32 rule) {
	u32 size_s = sol.solution.size();
	if (k >= size_s) k = size_s;
	if (k == 0 || k == 1) return;
	std::vector<u32> select_s(size_s, 0);
	std::iota(select_s.begin() + 1, select_s.end(), 1);
	std::random_device rd;
	Xoshiro::Xoshiro128ss gen(rd());
	std::shuffle(select_s.begin(), select_s.end(), gen);
	select_s.resize(k);
	select_s.emplace_back(select_s.front());
	for (u32 i{0}; i < k; i++) {
		u32 range_a = sol.solution[select_s[i]].path.size() - 2;
		u32 range_b = sol.solution[select_s[i + 1]].path.size() - 2;
		int n = epoch;
		while (n--) {
			u32 index_a{gen() % range_a + 1};
			u32 index_b{gen() % range_b + 1};
			int difload = sol.solution[select_s[i]].path[index_a]->demand - sol.solution[select_s[i + 1]].path[index_b]->demand;
			if (rule & FORCE) {
				sol.solution[select_s[i]].load -= difload;
				sol.solution[select_s[i + 1]].load += difload;
				sol.shash[sol.solution[select_s[i]].path[index_a]->seq] = sol.solution[select_s[i + 1]].seq;
				sol.shash[sol.solution[select_s[i + 1]].path[index_b]->seq] = sol.solution[select_s[i]].seq;
				std::swap(sol.solution[select_s[i + 1]].path[index_b], sol.solution[select_s[i]].path[index_a]);
				sol.solution[select_s[i + 1]].path_cumlength(1);
				sol.solution[select_s[i]].path_cumlength(1);
#ifdef PR
				sol.alltardiness = priority(sol);
#endif
				// break;
			}
			//} else if (sol.solution[select_s[i]].load - difload <= sol.solution[select_s[i]].capacity && sol.solution[select_s[i + 1]].load + difload <= sol.solution[select_s[i + 1]].capacity) {
			else {
				if (!sol.solution[select_s[i]].path[index_a - 1]->isdepot && sol.solution[select_s[i]].path[index_a - 1]->end > sol.solution[select_s[i + 1]].path[index_b]->end)
					continue;
				if (!sol.solution[select_s[i]].path[index_a + 1]->isdepot && sol.solution[select_s[i + 1]].path[index_b]->end > sol.solution[select_s[i]].path[index_a + 1]->end)
					continue;
				if (!sol.solution[select_s[i + 1]].path[index_b - 1]->isdepot && sol.solution[select_s[i + 1]].path[index_b - 1]->end > sol.solution[select_s[i]].path[index_a]->end)
					continue;
				if (!sol.solution[select_s[i + 1]].path[index_b + 1]->isdepot && sol.solution[select_s[i]].path[index_a]->end > sol.solution[select_s[i + 1]].path[index_b + 1]->end)
					continue;
				sol.solution[select_s[i]].load -= difload;
				sol.solution[select_s[i + 1]].load += difload;
				sol.shash[sol.solution[select_s[i]].path[index_a]->seq] = sol.solution[select_s[i + 1]].seq;
				sol.shash[sol.solution[select_s[i + 1]].path[index_b]->seq] = sol.solution[select_s[i]].seq;
				std::swap(sol.solution[select_s[i + 1]].path[index_b], sol.solution[select_s[i]].path[index_a]);
				sol.solution[select_s[i + 1]].path_cumlength(1);
				sol.solution[select_s[i]].path_cumlength(1);
#ifdef PR
				sol.alltardiness = priority(sol);
#endif
				// break;
			}
		}
	}
	sol.update();
}

bool OPS::onepointmove(Solution& s, Vehicle& r, const u32 a, const u32 b, u32 ctrl) {
	double saving0{100000000.0}, saving1{100000000.0};
	u32 cmd0{0}, flag{0};
	if (ctrl) cmd0 += FORCE;
	i64 saving{};  // 初始化保存插入前面和后面的节省距离
	double r_front_path{}, r_back_path{}, r_front_limit{}, r_back_limit{}, s_front_tardiness{}, s_back_tardiness{};
	r.precheck(cmd0);
	if (cmd0 & LOADS) return false;
	if (cmd0 & LENGTH)
		flag = 1;
	if (a < b) {  // a在b前面
		ALG::rotate(r.path.begin() + a, r.path.begin() + b, -1);
		if (!r.evaluate(r_front_path, r_front_limit, cmd0)) {  // 不可行
			if (flag == 1) {                                   // 长度
				if (r_front_limit < r.length) {
					r.cumlength = r_front_path;
					r.length = r_front_limit;
					return true;
				} else {
					ALG::rotate(r.path.begin() + a, r.path.begin() + b, 1);
					return false;
				}
			}
		} else {  // 可行
#ifdef PR
			s_front_tardiness = priority(s);
#endif

			saving0 = r_front_path - r.cumlength;
			saving0 = v_aim(saving0, s_front_tardiness - s.alltardiness);
		}
		std::swap(r.path[b], r.path[b - 1]);
		if (r.evaluate(r_back_path, r_back_limit, cmd0)) {  // 评估插入r2的后面
#ifdef PR
			s_back_tardiness = priority(s);
#endif

			saving1 = r_back_path - r.cumlength;
			saving1 = v_aim(saving1, s_back_tardiness - s.alltardiness);
		}
		if (saving0 < saving1) {
			// 插入前面更优
			// location = false;
			saving = static_cast<i64>(saving0 * 1000);
			if (saving >= 0 && !cmd0) {  // 如果节省距离大于0，说明前插入不可行，需要还原
				ALG::rotate(r.path.begin() + a, r.path.begin() + b + 1, 1);
				return false;  // 返回不可行
			}
			// 可行，撤销交换
			std::swap(r.path[b], r.path[b - 1]);
			r.cumlength = r_front_path;
			r.length = r_front_limit;
#ifdef PR
			s.alltardiness = s_front_tardiness;
#endif

			return true;
		} else {
			// 插入后面更优
			// location = true;
			saving = static_cast<i64>(saving1 * 1000);
			if (saving >= 0 && !cmd0) {  // 如果节省距离大于0，说明后插入不可行，需要还原
				ALG::rotate(r.path.begin() + a, r.path.begin() + b + 1, 1);
				return false;  // 返回不可行
			}
			// 可行
			r.cumlength = r_back_path;
			r.length = r_back_limit;
#ifdef PR
			s.alltardiness = s_back_tardiness;
#endif

			return true;
		}
	} else {  // a在b后面
		ALG::rotate(r.path.begin() + b, r.path.begin() + a + 1, 1);
		if (!r.evaluate(r_front_path, r_front_limit, cmd0)) {  // 不可行
			if (flag == 1) {                                   // 长度
				if (r_front_limit < r.length) {
					r.cumlength = r_front_path;
					r.length = r_front_limit;
					return true;
				} else {
					ALG::rotate(r.path.begin() + b, r.path.begin() + a + 1, -1);
					return false;
				}
			}
		} else {  // 可行
#ifdef PR
			s_front_tardiness = priority(s);
#endif

			saving0 = r_front_path - r.cumlength;
			saving0 = v_aim(saving0, s_front_tardiness - s.alltardiness);
		}
		std::swap(r.path[b], r.path[b + 1]);
		if (r.evaluate(r_back_path, r_back_limit, cmd0)) {  // 评估插入r2的后面
#ifdef PR
			s_back_tardiness = priority(s);
#endif

			saving1 = r_back_path - r.cumlength;
			saving1 = v_aim(saving1, s_back_tardiness - s.alltardiness);
		}
		if (saving0 < saving1) {
			// 插入前面更优
			// location = false;
			saving = static_cast<i64>(saving0 * 1000);
			if (saving >= 0 && !cmd0) {  // 如果节省距离大于0，说明前插入不可行，需要还原
				ALG::rotate(r.path.begin() + b + 1, r.path.begin() + a + 1, -1);
				return false;  // 返回不可行
			}
			// 可行，撤销交换
			std::swap(r.path[b], r.path[b + 1]);
			r.cumlength = r_front_path;
			r.length = r_front_limit;
#ifdef PR
			s.alltardiness = s_front_tardiness;
#endif

			return true;
		} else {
			// 插入后面更优
			// location = true;
			saving = static_cast<i64>(saving1 * 1000);
			if (saving >= 0 && !cmd0) {  // 如果节省距离大于0，说明后插入不可行，需要还原
				ALG::rotate(r.path.begin() + b + 1, r.path.begin() + a + 1, -1);
				return false;  // 返回不可行
			}
			// 可行
			r.cumlength = r_back_path;
			r.length = r_back_limit;
#ifdef PR
			s.alltardiness = s_back_tardiness;
#endif

			return true;
		}
	}
}

bool OPS::onepointmove(Solution& s, Vehicle& r1, Vehicle& r2, const u32 a, const u32 b, u32 ctrl) {
	Node* temp{r1.path[a]};  // 保存要插入的节点
	double saving0{100000000.0}, saving1{100000000.0};
	u32 cmd1{0}, cmd2{0}, flag{0};
	if (ctrl) cmd1 = cmd2 += FORCE;  // 是否强制
	i64 saving{};  // 初始化保存插入前面和后面的节省距离
	double r1_newpath{}, r1_newlimit{}, r2_front_path{}, r2_back_path{}, r2_front_limit{}, r2_back_limit{}, s_front_tardiness{-1.0}, s_back_tardiness{-1.0};
	// bool location{};         // 保存插入的位置
	// 不在同路径
	r1.precheck(cmd1), r2.precheck(cmd2);
	if (cmd1 & LOADS || cmd2 & LOADS)
		flag = 3;
	else if (cmd1 & LENGTH || cmd2 & LENGTH)
		flag = 1;
	r1.path.erase(r1.path.begin() + a);          // 从r1中删除要插入的节点
	r2.path.emplace(r2.path.begin() + b, temp);  // 在r2中插入节点
	if (!r1.evaluate(r1_newpath, r1_newlimit, cmd1) || !r2.evaluate(r2_front_path, r2_front_limit, cmd2)) {                 // 不可行
		if (flag == 3) {                                                                                                    // load
			if (r2.load - r1.load + temp->demand < 0) {
				r2.cumlength = r2_front_path;
				r2.length = r2_front_limit;
				r1.cumlength = r1_newpath;
				r1.length = r1_newlimit;
				r1.load -= temp->demand;
				r2.load += temp->demand;
#ifdef PR
				s.alltardiness = priority(s);
#endif

				return true;
			} else {
				r1.path.emplace(r1.path.begin() + a, temp);  // 还原r1路径
				r2.path.erase(r2.path.begin() + b);          // 还原r2路径
				return false;                                // 返回不可行
			}
		} else if (flag == 1) {  // 路径长度限制
			if (r2_front_limit < r2.Limit) {
				saving0 = r2_front_limit - r2.length;
			}
		}
	} else {  // 可行
#ifdef PR
		s_front_tardiness = priority(s);
#endif

		saving0 = r1_newpath + r2_front_path - r1.cumlength - r2.cumlength;
		saving0 = v_aim(saving0, s_front_tardiness - s.alltardiness);
	}
	std::swap(r2.path[b], r2.path[b + 1]);  // 交换r2中插入的节点和后面的节点
	if (!r2.evaluate(r2_back_path, r2_back_limit, cmd2)) {
		if (flag == 1) {
			if (r2_back_limit < r2.Limit) {
				saving1 = r2_back_limit - r2.length;
			}
		}
	} else {
#ifdef PR
		s_back_tardiness = priority(s);
#endif

		saving1 = r1_newpath + r2_back_path - r1.cumlength - r2.cumlength;
		saving1 = v_aim(saving1, s_back_tardiness - s.alltardiness);
	}
	if (saving0 < saving1) {
		// 插入前面更优
		// location = false;
		saving = static_cast<i64>(saving0 * 1000);
		if (saving >= 0 && !(cmd2 & FORCE)) {            // 如果节省距离大于0，说明前插入不可行，需要还原
			r1.path.emplace(r1.path.begin() + a, temp);  // 还原r1路径
			r2.path.erase(r2.path.begin() + b + 1);      // 还原r2路径
			return false;                                // 返回不可行
		}
		// 可行，撤销交换
		std::swap(r2.path[b], r2.path[b + 1]);
		r2.cumlength = r2_front_path;
		r2.length = r2_front_limit;
		r1.cumlength = r1_newpath;
		r1.length = r1_newlimit;
		r1.load -= temp->demand;
		r2.load += temp->demand;
#ifdef PR
		if (s_front_tardiness < 0.0)
			s.alltardiness = priority(s);
		else
			s.alltardiness = s_front_tardiness;
#endif

		return true;
	} else {
		// 插入后面更优
		// location = true;
		saving = static_cast<i64>(saving1 * 1000);
		if (saving >= 0 && !(cmd2 & FORCE)) {            // 如果节省距离大于0，说明后插入不可行，需要还原
			r1.path.emplace(r1.path.begin() + a, temp);  // 还原r1路径
			r2.path.erase(r2.path.begin() + b + 1);      // 还原r2路径
			return false;                                // 返回不可行
		}
		// 可行
		r2.cumlength = r2_back_path;
		r2.length = r2_back_limit;
		r1.cumlength = r1_newpath;
		r1.length = r1_newlimit;
		r1.load -= temp->demand;
		r2.load += temp->demand;
#ifdef PR
		if (s_back_tardiness < 0.0)
			s.alltardiness = priority(s);
		else
			s.alltardiness = s_back_tardiness;
#endif

		return true;
	}
}

bool OPS::reverse(Solution& s, Vehicle& r, const u32 f, const u32 t, u32 ctrl) {
	i64 saving{};
	double r_path{}, r_path_limit{}, s_tardiness{-1.0};
	// bool improved{false};
	u32 cmd0{}, flag{};
	if (ctrl) cmd0 += FORCE;
	r.precheck(cmd0);
	if (cmd0 & LOADS)
		return false;
	else if (cmd0 & LENGTH)
		flag = 1;
	// 倒转路径中从f到t之间的元素
	std::reverse(r.path.begin() + f, r.path.begin() + t);
	// 如果倒转后的路径评估成功
	if (!r.evaluate(r_path, r_path_limit, cmd0)) {  // 不可行
		if (flag == 1) {                            // 长度
			// if (r_path_limit < r.length) {
			saving = static_cast<i64>((r_path_limit - r.length) * 1000);
			// improved = true;
			//}
		}
	} else {  // 可行
#ifdef PR
		s_tardiness = priority(s);
#endif

		saving = static_cast<i64>(v_aim(r_path - r.cumlength, s_tardiness - s.alltardiness) * 1000);
		// improved = true;
	}
	if (cmd0 & FORCE || saving < 0) {
		r.cumlength = r_path;
		r.length = r_path_limit;
#ifdef PR
		if (s_tardiness < 0.0)
			s.alltardiness = priority(s);
		else
			s.alltardiness = s_tardiness;
#endif

		return true;
	}
	std::reverse(r.path.begin() + f, r.path.begin() + t);
	return false;
}

bool OPS::swapmove(Solution& s, Vehicle& r1, Vehicle& r2, const u32 a, const u32 b, u32 ctrl) {
	u32 cmd1{0}, cmd2{0}, flag{0};
	if (ctrl) cmd1 = cmd2 += FORCE;  // 是否强制
	i64 saving{};                    // 初始化保存插入前面和后面的节省距离
	double r1_newpath{}, r1_newlimit{}, r2_new_path{}, r2_limit{}, s_tardiness{-1.0};
	int difload = r1.path[a]->demand - r2.path[b]->demand;
	if (r1.seq == r2.seq) {  // 同车
		r1.precheck(cmd1);
		if (cmd1 & LOADS)
			return false;
		else if (cmd1 & LENGTH)
			flag = 1;
		std::swap(r1.path[a], r1.path[b]);  // 交换路径中的两个位置
		if (!r1.evaluate(r1_newpath, r1_newlimit, cmd1)) {  // 不可行
			if (flag == 1) {                                // 长度
				// if (r1_newlimit < r1.length) {
				saving = static_cast<i64>((r1_newlimit - r1.length) * 1000);
				// improved = true;
				//}
			}
		} else {  // 可行
#ifdef PR
			s_tardiness = priority(s);
#endif

			saving = static_cast<i64>(v_aim(r1_newpath - r1.cumlength, s_tardiness - s.alltardiness) * 1000);
			// improved = true;
		}
		if (cmd1 & FORCE || saving < 0) {
			r1.cumlength = r1_newpath;
			r1.length = r1_newlimit;
#ifdef PR
			if (s_tardiness < 0.0)
				s.alltardiness = priority(s);
			else
				s.alltardiness = s_tardiness;
#endif

			return true;
		}
		std::swap(r1.path[a], r1.path[b]);     // 恢复原始路径
		return false;                          // 返回交换失败
	} else {                                   // 不同车
		r1.precheck(cmd1), r2.precheck(cmd2);
		if (cmd1 & LOADS || cmd2 & LOADS)
			flag = 3;
		else if (cmd1 & LENGTH || cmd2 & LENGTH)
			flag = 1;
		if (flag == 3) {
			if (difload == 0) return false;
			// if (difload > 0) {
			//	if (r2.load - r1.load + difload >= 0) return false;
			// } else {
			//	if (r2.load - r1.load + difload <= 0) return false;
			// }
			if (difload > 0) {
				if (r2.load + difload > r2.capacity) return false;
			} else {
				if (r1.load - difload > r1.capacity) return false;
			}
			r1.load -= difload;
			r2.load += difload;
			std::swap(r1.path[a], r2.path[b]);
			r1.update_allength();
			r2.update_allength();
#ifdef PR
			s.alltardiness = priority(s);
#endif

			return true;
		}
		std::swap(r1.path[a], r2.path[b]);     // 交换路径中的两个位置
		if (!r1.evaluate(r1_newpath, r1_newlimit, cmd1) || !r2.evaluate(r2_new_path, r2_limit, cmd2)) {  // 不可行
			if (flag == 1) {                                                                             // 路径长度限制
				saving = static_cast<i64>((r2_limit + r1_newlimit - r2.length - r1.length) * 1000);
			}
		} else {  // 可行
#ifdef PR
			s_tardiness = priority(s);
#endif

			saving = static_cast<i64>(v_aim(r1_newpath + r2_new_path - r1.cumlength - r2.cumlength, s_tardiness - s.alltardiness) * 1000);
		}
		if (cmd1 & FORCE || saving < 0) {
			r1.cumlength = r1_newpath;
			r1.length = r1_newlimit;
			r2.length = r2_limit;
			r2.cumlength = r2_new_path;
			r1.load -= difload;
			r2.load += difload;
#ifdef PR
			if (s_tardiness < 0.0)
				s.alltardiness = priority(s);
			else
				s.alltardiness = s_tardiness;
#endif

			return true;  // 返回交换成功
		}
		std::swap(r1.path[a], r2.path[b]);  // 恢复原始路径
		return false;                       // 返回交换失败
	}
}
bool OPS::oropt(Solution& s, Vehicle& r, const u32 f, const u32 t, const u32 len, u32 ctrl) {
	double saving0{100000000.0}, saving1{100000000.0};
	u32 cmd0{0}, flag{0};
	if (ctrl) cmd0 += FORCE;
	i64 saving{};  // 初始化保存插入前面和后面的节省距离
	double r_front_path{}, r_back_path{}, r_front_limit{}, r_back_limit{}, s_front_tardiness{-1.0}, s_back_tardiness{-1.0};
	r.precheck(cmd0);
	if (cmd0 & LOADS)
		return false;
	else if (cmd0 & LENGTH)
		flag = 1;
	// bool location{};
	if (f < t) {                                                         // f在t前面
		if (t - f == len) {                                              // 相邻
			ALG::rotate(r.path.begin() + f, r.path.begin() + t + 1, 1);  // 将路径中f到f1的部分旋转
			// r_back = r.path_cumlength();
			//  如果路径评估成功，则计算节约值
			if (!r.evaluate(r_back_path, r_back_limit, cmd0)) {  // 不可行
				if (flag == 1) {                                 // 长度
					if (r_back_limit < r.length) {
						saving1 = r_back_limit - r.length;
					}
				}
			} else {  // 可行
#ifdef PR
				s_back_tardiness = priority(s);
#endif

				saving1 = r_back_path - r.cumlength;
				saving1 = v_aim(saving1, s_back_tardiness - s.alltardiness);
			}
			// 如果节约值大于0，则表示路径优化失败
			saving = static_cast<i64>(saving1 * 1000);
			if (saving >= 0 && !cmd0) {
				// location = false;
				ALG::rotate(r.path.begin() + f, r.path.begin() + t + 1, -1);  //  还原
				return false;
			}
			// location = 1;
			// saving = saving1;
			r.cumlength = r_back_path;
			r.length = r_back_limit;
#ifdef PR
			if (s_back_tardiness < 0.0)
				s.alltardiness = priority(s);
			else
				s.alltardiness = s_back_tardiness;
#endif
			return true;
		}
		// 向左旋转r的路径
		ALG::rotate(r.path.begin() + f, r.path.begin() + t, -len);
		// r_front = r.path_cumlength();
		//  前插
		if (!r.evaluate(r_front_path, r_front_limit, cmd0)) {  // 不可行
			if (flag == 1) {                                   // 长度
				if (r_front_limit < r.length) {
					saving0 = r_front_limit - r.length;
				}
			}
		} else {  // 可行
#ifdef PR
			s_front_tardiness = priority(s);
#endif

			saving0 = r_front_path - r.cumlength;
			saving0 = v_aim(saving0, s_front_tardiness - s.alltardiness);
		}
		ALG::rotate(r.path.begin() + t - len, r.path.begin() + t + 1, 1);
		// r_back = r.path_cumlength();
		//  后插
		if (!r.evaluate(r_back_path, r_back_limit, cmd0)) {  // 评估插入r2的后面
			if (flag == 1) {                                 // 长度
				if (r_back_limit < r.length) {
					saving1 = r_back_limit - r.length;
				}
			}
		} else {  // 可行
#ifdef PR
			s_back_tardiness = priority(s);
#endif

			saving1 = r_back_path - r.cumlength;
			saving1 = v_aim(saving1, s_back_tardiness - s.alltardiness);
		}
		if (saving0 < saving1) {
			saving = static_cast<int>(saving0 * 1000);
			if (saving >= 0 && !cmd0) {  // 前插不可行
				ALG::rotate(r.path.begin() + f, r.path.begin() + t + 1, len);
				return false;
			}
			ALG::rotate(r.path.begin() + t - len, r.path.begin() + t + 1, -1);
			r.cumlength = r_front_path;
			r.length = r_front_limit;
#ifdef PR
			if (s_front_tardiness < 0.0)
				s.alltardiness = priority(s);
			else
				s.alltardiness = s_front_tardiness;
#endif

			return true;
		} else {
			saving = static_cast<i64>(saving1 * 1000);
			if (saving >= 0 && !cmd0) {  // 后插不可行
				ALG::rotate(r.path.begin() + f, r.path.begin() + t + 1, len);
				return false;
			}
			r.cumlength = r_back_path;
			r.length = r_back_limit;
#ifdef PR
			if (s_back_tardiness < 0.0)
				s.alltardiness = priority(s);
			else
				s.alltardiness = s_back_tardiness;
#endif

			return true;
		}
	} else {               // f在t后面
		if (f - t == 1) {  // 相邻
			ALG::rotate(r.path.begin() + t, r.path.begin() + f + len, -1);
			// r_front = r.path_cumlength();
			if (!r.evaluate(r_front_path, r_front_limit, cmd0)) {  // 不可行
				if (flag == 1) {                                   // 长度
					if (r_front_limit < r.length) {
						saving0 = r_front_limit - r.length;
					}
				}
			} else {  // 可行
#ifdef PR
				s_front_tardiness = priority(s);
#endif

				saving0 = r_front_path - r.cumlength;
				saving0 = v_aim(saving0, s_front_tardiness - s.alltardiness);
			}
			saving = static_cast<i64>(saving0 * 1000);
			if (saving >= 0 && !cmd0) {
				// location = false;
				ALG::rotate(r.path.begin() + t, r.path.begin() + f + len, 1);  //  还原
				return false;
			}
			// location = 1;
			// saving = saving1;
			r.cumlength = r_front_path;
			r.length = r_front_limit;
#ifdef PR
			if (s_front_tardiness < 0.0)
				s.alltardiness = priority(s);
			else
				s.alltardiness = s_front_tardiness;
#endif

			return true;
		}
		// 向右旋转r的路径
		ALG::rotate(r.path.begin() + t, r.path.begin() + f + len, len);
		// r_front = r.path_cumlength();
		if (!r.evaluate(r_front_path, r_front_limit, cmd0)) {  // 不可行
			if (flag == 1) {                                   // 长度
				if (r_front_limit < r.length) {
					saving0 = r_front_limit - r.length;
				}
			}
		} else {  // 可行
#ifdef PR
			s_front_tardiness = priority(s);
#endif

			saving0 = r_front_path - r.cumlength;
			saving0 = v_aim(saving0, s_front_tardiness - s.alltardiness);
		}
		ALG::rotate(r.path.begin() + t, r.path.begin() + t + len + 1, 1);
		// r_back = r.path_cumlength();
		if (!r.evaluate(r_back_path, r_back_limit, cmd0)) {  // 评估插入r2的后面
			if (flag == 1) {                                 // 长度
				if (r_back_limit < r.length) {
					saving1 = r_back_limit - r.length;
				}
			}
		} else {  // 可行
#ifdef PR
			s_back_tardiness = priority(s);
#endif

			saving1 = r_back_path - r.cumlength;
			saving1 = v_aim(saving1, s_back_tardiness - s.alltardiness);
		}
		if (saving0 < saving1) {
			saving = static_cast<int>(saving0 * 1000);
			if (saving >= 0 && !cmd0) {  // 前插不可行
				ALG::rotate(r.path.begin() + t + 1, r.path.begin() + f + len, -len);
				return false;
			}
			ALG::rotate(r.path.begin() + t, r.path.begin() + t + len + 1, -1);
			r.cumlength = r_front_path;
			r.length = r_front_limit;
#ifdef PR
			if (s_front_tardiness < 0.0)
				s.alltardiness = priority(s);
			else
				s.alltardiness = s_front_tardiness;
#endif

			return true;
		} else {
			saving = static_cast<i64>(saving1 * 1000);
			if (saving >= 0 && !cmd0) {  // 后插不可行
				ALG::rotate(r.path.begin() + t + 1, r.path.begin() + f + len, -len);
				return false;
			}
			r.cumlength = r_back_path;
			r.length = r_back_limit;
#ifdef PR
			if (s_back_tardiness < 0.0)
				s.alltardiness = priority(s);
			else
				s.alltardiness = s_back_tardiness;
#endif

			return true;
		}
	}
}

bool OPS::oropt(Solution& s, Vehicle& r1, Vehicle& r2, const u32 f, const u32 t, const u32 len, u32 ctrl) {
	const u32 f1{f + len}, t1{t + len};
	double saving0{100000000.0}, saving1{100000000.0};
	u32 cmd1{0}, cmd2{0}, flag{0}, difload{};
	if (ctrl) cmd1 = cmd2 += FORCE;  // 是否强制
	i64 saving{};  // 初始化保存插入前面和后面的节省距离
	double r1_newpath{}, r1_newlimit{}, r2_front_path{}, r2_back_path{}, r2_front_limit{}, r2_back_limit{}, s_front_tardiness{-1.0}, s_back_tardiness{-1.0};
	// bool location{};
	//  从r1中提取要移动的节点
	std::vector<Node*> temp(r1.path.begin() + f, r1.path.begin() + f1);
	r1.precheck(cmd1), r2.precheck(cmd2);
	if (cmd1 & LOADS || cmd2 & LOADS)
		flag = 3;
	else if (cmd1 & LENGTH || cmd2 & LENGTH)
		flag = 1;
	if (len == 2) {
		difload += temp[0]->demand + temp[1]->demand;
	} else if (len == 3) {
		difload += temp[0]->demand + temp[1]->demand + temp[2]->demand;
	} else if (len == 4) {
		difload += temp[0]->demand + temp[1]->demand + temp[2]->demand + temp[3]->demand;
	}
	// 从r1中移除要移动的节点
	r1.path.erase(r1.path.begin() + f, r1.path.begin() + f1);
	// 将节点插入到r2的指定位置
	r2.path.insert(r2.path.begin() + t, temp.begin(), temp.end());
	// 计算r1的路径长度
	if (flag == 3) {  // load
		if (r2.load - r1.load + difload < 0) {
			r1.update_allength();
			r2.update_allength();
			r1.load -= difload;
			r2.load += difload;
#ifdef PR
			s.alltardiness = priority(s);
#endif

			return true;
		} else {
			r1.path.insert(r1.path.begin() + f, temp.begin(), temp.end());
			r2.path.erase(r2.path.begin() + t, r2.path.begin() + t1);
			return false;  // 返回不可行
		}
	}
	if (!r1.evaluate(r1_newpath, r1_newlimit, cmd1) || !r2.evaluate(r2_front_path, r2_front_limit, cmd2)) {  // 不可行
		if (flag == 1) {                                                                                     // 路径长度限制
			if (r2_front_limit < r2.length) {
				saving0 = r2_front_limit - r2.length;
			}
		}
	} else {  // 可行
#ifdef PR
		s_front_tardiness = priority(s);
#endif

		saving0 = r1_newpath + r2_front_path - r1.cumlength - r2.cumlength;
		saving0 = v_aim(saving0, s_front_tardiness - s.alltardiness);
	}
	// 向右旋转r2的路径
	ALG::rotate(r2.path.begin() + t, r2.path.begin() + t + len + 1, 1);
	// 计算r2的路径长度后
	// r2_back = r2.path_cumlength();
	// 评估将节点插入r2的后面的情况
	if (!r2.evaluate(r2_back_path, r2_back_limit, cmd2)) {
		if (flag == 1) {  // 长度限制
			if (r2_back_limit < r2.length) {
				saving1 = r2_back_limit - r2.length;
			}
		}
	} else {  // 可行正常
#ifdef PR
		s_back_tardiness = priority(s);
#endif

		saving1 = r1_newpath + r2_back_path - r1.cumlength - r2.cumlength;
		saving1 = v_aim(saving1, s_back_tardiness - s.alltardiness);
	}
	if (saving0 < saving1) {
		// 插入前面更优
		// location = false;
		saving = static_cast<i64>(saving0 * 1000);
		if (saving >= 0 && !(cmd2 & FORCE)) {  // 如果节省距离大于0，说明前插入不可行，需要还原
			r1.path.insert(r1.path.begin() + f, temp.begin(), temp.end());
			r2.path.erase(r2.path.begin() + t + 1, r2.path.begin() + t1 + 1);
			return false;  // 返回不可行
		}
		// 可行，撤销交换
		ALG::rotate(r2.path.begin() + t, r2.path.begin() + t + len + 1, -1);
		r2.cumlength = r2_front_path;
		r2.length = r2_front_limit;
		r1.cumlength = r1_newpath;
		r1.length = r1_newlimit;
		r1.load -= difload;
		r2.load += difload;
#ifdef PR
		if (s_front_tardiness < 0.0)
			s.alltardiness = priority(s);
		else
			s.alltardiness = s_front_tardiness;
#endif

		return true;
	} else {
		// 插入后面更优
		// location = true;
		saving = static_cast<i64>(saving1 * 1000);
		if (saving >= 0 && !(cmd2 & FORCE)) {  // 如果节省距离大于0，说明后插入不可行，需要还原
			r1.path.insert(r1.path.begin() + f, temp.begin(), temp.end());
			r2.path.erase(r2.path.begin() + t + 1, r2.path.begin() + t1 + 1);
			return false;  // 返回不可行
		}
		// 可行
		r2.cumlength = r2_back_path;
		r2.length = r2_back_limit;
		r1.cumlength = r1_newpath;
		r1.length = r1_newlimit;
		r1.load -= difload;
		r2.load += difload;
#ifdef PR
		if (s_back_tardiness < 0.0)
			s.alltardiness = priority(s);
		else
			s.alltardiness = s_back_tardiness;
#endif

		return true;
	}
}

/// @brief 判断两个车辆的路径是否可以交换其中的三个节点，计算路径长度变化
/// @param r1
/// @param r2
/// @param a
/// @param b
/// @param saving
/// @return
bool OPS::arcnode(Solution& s, Vehicle& r1, Vehicle& r2, const u32 a, const u32 b, u32 ctrl) {
	u32 cmd1{0}, cmd2{0}, flag{0};
	if (ctrl) cmd1 = cmd2 += FORCE;               // 是否强制
	i64 saving{};                                 // 保存交换节点后的路径长度
	Node* node{r1.path[a + 1]};                   // 保存需要交换的节点
	int difload = r1.path[a + 1]->demand + r1.path[a]->demand - r2.path[b]->demand;  // 保存交换节点后的负载变化
	double r1_newpath{}, r1_newlimit{}, r2_new_path{}, r2_limit{}, s_tardiness{-1.0};
	if (r1.seq == r2.seq) {                       // 如果两个车辆的路径相同
		r1.precheck(cmd1);
		if (cmd1 & LOADS)
			return false;
		if (cmd1 & LENGTH)
			flag = 1;
		if (a < b) {                              // 如果a在b前面
			std::swap(r1.path[a], r1.path[b]);    // 交换节点
			for (auto it{a + 1}; it < b; it++) {  // 调整路径
				std::swap(r1.path[it], r1.path[it + 1]);
			}
			if (!r1.evaluate(r1_newpath, r1_newlimit, cmd1)) {  // 不可行
				if (flag == 1) {                                // 长度
					saving = static_cast<i64>((r1_newlimit - r1.length) * 1000);
				}
			} else {  // 可行
#ifdef PR
				s_tardiness = priority(s);
#endif

				saving = static_cast<i64>(v_aim(r1_newpath - r1.cumlength, s_tardiness - s.alltardiness) * 1000);
			}
			if (cmd1 & FORCE || saving < 0) {
				r1.cumlength = r1_newpath;
				r1.length = r1_newlimit;
#ifdef PR
				if (s_tardiness < 0.0)
					s.alltardiness = priority(s);
				else
					s.alltardiness = s_tardiness;
#endif

				return true;
			}
			std::swap(r1.path[a], r1.path[b]);  // 恢复节点交换前的路径
			for (auto it{b - 1}; it > a; it--) {
				std::swap(r1.path[it], r1.path[it - 1]);
			}
			return false;                       // 返回交换失败
		} else {                                // 如果a在b后面
			std::swap(r1.path[a], r1.path[b]);  // 交换节点
			for (auto it{a}; it > b; it--) {    // 调整路径
				std::swap(r1.path[it], r1.path[it + 1]);
			}
			if (!r1.evaluate(r1_newpath, r1_newlimit, cmd1)) {  // 不可行
				if (flag == 1) {                                // 长度
					saving = static_cast<i64>((r1_newlimit - r1.length) * 1000);
				}
			} else {  // 可行
#ifdef PR
				s_tardiness = priority(s);
#endif

				saving = static_cast<i64>(v_aim(r1_newpath - r1.cumlength, s_tardiness - s.alltardiness) * 1000);
			}
			if (cmd1 & FORCE || saving < 0) {
				r1.cumlength = r1_newpath;
				r1.length = r1_newlimit;
#ifdef PR
				if (s_tardiness < 0.0)
					s.alltardiness = priority(s);
				else
					s.alltardiness = s_tardiness;
#endif

				return true;
			}
			std::swap(r1.path[a + 1], r1.path[b + 1]);  // 恢复节点交换前的路径
			for (auto it{b}; it < a; it++) {
				std::swap(r1.path[it], r1.path[it + 1]);
			}
			return false;  // 返回交换失败
		}
	} else {                                     // 如果两个车辆的路径不同
		r1.precheck(cmd1), r2.precheck(cmd2);
		if (cmd1 & LOADS || cmd2 & LOADS)
			return false; // ? 
//          flag = 3; //可能会死循环
		else if (cmd1 & LENGTH || cmd2 & LENGTH)
			flag = 1;
		if (flag == 3) {
			if (difload == 0) return false;
			if (difload > 0) {
				if (r2.load - r1.load + difload >= 0) return false;
			} else {
				if (r2.load - r1.load + difload <= 0) return false;
			}
			r1.load -= difload;
			r2.load += difload;
			std::swap(r1.path[a], r2.path[b]);               // 交换节点
			r1.path.erase(r1.path.begin() + a + 1);          // 调整路径
			r2.path.emplace(r2.path.begin() + b + 1, node);  // 调整路径
			r1.update_allength();
			r2.update_allength();
#ifdef PR
			s.alltardiness = priority(s);
#endif

			return true;
		}
		std::swap(r1.path[a], r2.path[b]);               // 交换节点
		r1.path.erase(r1.path.begin() + a + 1);          // 调整路径
		r2.path.emplace(r2.path.begin() + b + 1, node);  // 调整路径
		if (!r1.evaluate(r1_newpath, r1_newlimit, cmd1) || !r2.evaluate(r2_new_path, r2_limit, cmd2)) {  // 不可行
			if (flag == 1) {                                                                             // 路径长度限制
				saving = static_cast<i64>((r2_limit + r1_newlimit - r2.length - r1.length) * 1000);
			}
		} else {  // 可行
#ifdef PR
			s_tardiness = priority(s);
#endif

			saving = static_cast<i64>(v_aim(r1_newpath + r2_new_path - r1.cumlength - r2.cumlength, s_tardiness - s.alltardiness) * 1000);
		}
		if (cmd1 & FORCE || saving < 0) {
			r1.cumlength = r1_newpath;
			r1.length = r1_newlimit;
			r2.length = r2_limit;
			r2.cumlength = r2_new_path;
			r1.load -= difload;
			r2.load += difload;
#ifdef PR
			if (s_tardiness < 0.0)
				s.alltardiness = priority(s);
			else
				s.alltardiness = s_tardiness;
#endif

			return true;  // 返回交换成功
		}
		std::swap(r1.path[a], r2.path[b]);  // 恢复节点交换前的路径
		r1.path.emplace(r1.path.begin() + a + 1, node);
		r2.path.erase(r2.path.begin() + b + 1);
		return false;  // 返回交换失败
	}
}

bool OPS::arcswap(Solution& s, Vehicle& r1, Vehicle& r2, float c, const u32 a, const u32 b, u32 ctrl) {
	i64 saving{};
	int difload = r1.path[a]->demand + r1.path[a + 1]->demand - r2.path[b]->demand - r2.path[b + 1]->demand;
	u32 cmd1{0}, cmd2{0}, flag{0};
	if (ctrl) cmd1 = cmd2 += FORCE;  // 是否强制
	double r1_newpath{}, r1_newlimit{}, r2_new_path{}, r2_limit{}, s_tardiness{-1.0};
	if (r1.seq == r2.seq) {                         // 同车
		r1.precheck(cmd1);
		if (cmd1 & LOADS)
			return false;
		else if (cmd1 & LENGTH)
			flag = 1;
		std::swap(r1.path[a], r1.path[b]);          // 交换路径中的两个位置
		std::swap(r1.path[a + 1], r1.path[b + 1]);  // 交换路径中的两个位置
		if (!r1.evaluate(r1_newpath, r1_newlimit, cmd1)) {  // 不可行
			if (flag == 1) {                                // 长度
				saving = static_cast<i64>((r1_newlimit - r1.length) * 1000);
			}
		} else {  // 可行
#ifdef PR
			s_tardiness = priority(s);
#endif
			saving = static_cast<i64>(v_aim(r1_newpath - r1.cumlength, s_tardiness - s.alltardiness) * 1000);
		}
		if (cmd1 & FORCE || saving < 0) {
			r1.cumlength = r1_newpath;
			r1.length = r1_newlimit;
#ifdef PR
			if (s_tardiness < 0.0)
				s.alltardiness = priority(s);
			else
				s.alltardiness = s_tardiness;
#endif

			return true;
		}
		std::swap(r1.path[a], r1.path[b]);          // 恢复原始路径
		std::swap(r1.path[a + 1], r1.path[b + 1]);  // 恢复原始路径
		return false;                               // 返回交换失败
	} else {                                        // 不同车
		r1.precheck(cmd1), r2.precheck(cmd2);
		if (cmd1 & LOADS || cmd2 & LOADS)
			flag = 3;
		else if (cmd1 & LENGTH || cmd2 & LENGTH)
			flag = 1;
		if (flag == 3) {
			if (difload == 0) return false;
			if (difload > 0) {
				if (r2.load - r1.load + difload >= 0) return false;
			} else {
				if (r2.load - r1.load + difload <= 0) return false;
			}
			r1.load -= difload;
			r2.load += difload;
			std::swap(r1.path[a], r2.path[b]);          // 交换路径中的两个位置
			std::swap(r1.path[a + 1], r2.path[b + 1]);  // 交换路径中的两个位置
			r1.update_allength();
			r2.update_allength();
#ifdef PR
			s.alltardiness = priority(s);
#endif

			return true;
		}
		std::swap(r1.path[a], r2.path[b]);                             // 交换路径中的两个位置
		std::swap(r1.path[a + 1], r2.path[b + 1]);                     // 交换路径中的两个位置
		if (!r1.evaluate(r1_newpath, r1_newlimit, cmd1) || !r2.evaluate(r2_new_path, r2_limit, cmd2)) {  // 不可行
			if (flag == 1) {                                                                             // 路径长度限制
				saving = static_cast<i64>((r2_limit + r1_newlimit - r2.length - r1.length) * 1000);
			}
		} else {  // 可行
#ifdef PR
			s_tardiness = priority(s);
#endif
			saving = static_cast<i64>(v_aim(r1_newpath + r2_new_path - r1.cumlength - r2.cumlength, s_tardiness - s.alltardiness) * 1000);
		}
		if (cmd1 & FORCE || saving < 0) {
			r1.cumlength = r1_newpath;
			r1.length = r1_newlimit;
			r2.length = r2_limit;
			r2.cumlength = r2_new_path;
			r1.load -= difload;
			r2.load += difload;
#ifdef PR
			if (s_tardiness < 0.0)
				s.alltardiness = priority(s);
			else
				s.alltardiness = s_tardiness;
#endif

			return true;  // 返回交换成功
		}
		std::swap(r1.path[a], r2.path[b]);          // 恢复原始路径
		std::swap(r1.path[a + 1], r2.path[b + 1]);  // 恢复原始路径
		return false;                               // 返回交换失败
	}
}