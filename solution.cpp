#include "solution.hpp"

#include <algorithm>
#include <vector>
// #include <set>

#include "algorithm.hpp"


Solution nassign(std::vector<Node*> customers, std::vector<Node*> depots, const u32 maxload, const u32 routes, u32& ctrl) {
	Solution solution;
	if (depots.size() == 1) {  // 单厂站
		solution.multi = false;
		solution.maxvehicle = routes;
		Node* depot{depots.front()};
		Vehicle vehicle(depot, maxload, 0);  // 初始路线
		// 按离厂站距离排序
		std::sort(customers.begin(), customers.end(), [](const Node* a, const Node* b) {
			if (a->end >= b->end)
				return a->demand > b->demand;
			else
				return false;
		});
		solution.add(vehicle, routes);
		u32 maxcust = customers.size();
		for (u32 i{0};; i++) {
			if (i < maxcust)
				solution.solution[i % routes].path.emplace_back(customers[i]);
			else
				break;
		}
		for (auto& v : solution.solution) {
			v.path.emplace_back(depot);
			v.path_cumlength(true);
		}
		/*
		for (auto& node : customers) {
		    if (!vehicle.move(node)) {             // 达到最大
		        vehicle.path.emplace_back(depot);  // 返回厂站
		        vehicle.path_cumlength(true);      // 计算路径长度。
		        solution.add(vehicle);             // 加入到答案。
		        vehicle.clear(depot, 0);           // 清空
		        vehicle.move(node);                // 补上
		    }
		}
		if (!vehicle.path.empty()) {           // 最后的客户
		    vehicle.path.emplace_back(depot);  // 返回厂站
		    vehicle.path_cumlength(true);      // 计算路径长度。
		    solution.add(vehicle);             // 加入到答案。
		    vehicle.clear(depot, 0);           // 清空
		}
		*/
	}
	/*if (solution.solution.size() < routes) {               // 加车
	    if (depot_num == 1) {
	        Vehicle vehicle(depots.back(), maxload, num);  // 初始路线
	        while (solution.solution.size() == routes) {
	            solution.add(vehicle);                     // 加入到答案。
	            vehicle.clear(depots.back(), num);         // 清空
	        }
	    } else {
	        Vehicle vehicle(depots.front(), maxload, num);  // 初始路线
	        for (auto& d : depots) {
	            vehicle.clear(d, num);                      // 清空
	            solution.add(vehicle);                      // 加入到答案。
	            if (solution.solution.size() == routes) break;
	        }
	    }
	}
	if (solution.solution.size() > routes) {  // 减少车辆
	    if (depot_num != 1 && !seq.empty()) {
	        std::sort(seq.begin(), seq.end(), [&solution](u32& a, u32& b) -> bool { return solution.solution[a].path.size() > solution.solution[b].path.size(); });
	        bool flag{true};
	        u32 size = solution.solution.size();
	        while (size > routes && flag) {
	            flag = false;
	            for (u32 j{0}, m = seq.size() - 1; j < m; j++) {
	                for (u32 i{1}, n = solution.solution[seq.back()].path.size() - 1; i < n; i++) {
	                    if (solution.solution[seq[j]].load + solution.solution[seq.back()].path[i]->demand <= solution.solution[seq[j]].capacity) {
	                        solution.solution[seq[j]].path.emplace(solution.solution[seq[j]].path.end() - 1, solution.solution[seq.back()].path[i]);
	                        solution.solution[seq[j]].load += solution.solution[seq.back()].path[i]->demand;
	                        solution.solution[seq.back()].path[i] = nullptr;
	                        flag = true;
	                    }
	                }
	                solution.solution[seq.back()].path.erase(std::remove(solution.solution[seq.back()].path.begin(), solution.solution[seq.back()].path.end(), nullptr), solution.solution[seq.back()].path.end());
	                if (solution.solution[seq.back()].path.size() <= 2) {
	                    seq.pop_back();
	                    size--;
	                    break;
	                }
	            }
	        }
	    } else {

	    }
	}
	// 删除空车
	solution.solution.erase(std::remove_if(solution.solution.begin(), solution.solution.end(), [](Vehicle& a) -> bool {if(a.path.size() <= 2) return true; else return false; }), solution.solution.end());
	*/
	// 重建序号
	solution.update_seq();
	solution.alltardiness = 0.0;
	solution.update();
	solution.evaluate(ctrl);
	// hash
	// for (u32 i = 0; i < solution.solution.size(); i++) {
	//	for (u32 j = 1; j < solution.solution[i].path.size() - 1; j++) {                                          // 排除厂站
	//		solution.shash.emplace(solution.solution[i].path[j]->seq, solution.solution[i].seq);                  // 建立hash 查找表
	//	}
	//}
	return solution;
}

Solution assign(std::vector<Node*> customers, std::vector<Node*> depots, const u32 maxload, const u32 routes, u32& ctrl) {
	Solution solution;
	solution.multi = false;
	solution.maxvehicle = routes;
	Node* depot{depots.front()};
	Vehicle vehicle(depot, maxload, 0);  // 初始路线
	// 按离厂站距离排序
	std::sort(customers.begin(), customers.end(), [](const Node* a, const Node* b) -> bool {
		return a->demand > b->demand;
	});
	solution.add(vehicle, routes);
	u32 maxcust = customers.size();
	for (u32 i{0};; i++) {
		if (i < maxcust) {
			std::sort(solution.solution.begin(), solution.solution.end(), [](Vehicle& a, Vehicle& b) -> bool { return a.load > b.load; });
			solution.solution.back().path.emplace_back(customers[i]);
			solution.solution.back().load += customers[i]->demand;
		} else
			break;
	}
	for (auto& v : solution.solution) {
		v.path.emplace_back(depot);
		v.path_cumlength(true);
	}
	solution.update_seq();
	solution.alltardiness = priority(solution);
	solution.update();
	solution.evaluate(ctrl);
	return solution;
}
