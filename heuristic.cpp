
#include "heuristic.hpp"

#include <algorithm>
#include <random>
#include <set>
#include <utility>
#include <vector>

#include "NSearch.hpp"
#include "algorithm.hpp"
#include "node.hpp"
#include "operator.hpp"
#include "solution.hpp"

// SA算法
void HLSVND::init(std::vector<Node*>& node, std::vector<Node*>& depot, std::vector<Node*>& customer, const u32 depot_num, u32 maxload, u32 routes) {
	// std::vector<Node*> depots, custers;
	// custers.assign(nodes.begin(), nodes.end() - depot_num);
	// depots.assign(nodes.end() - depot_num, nodes.end());  // 厂站必须在节点的末尾
	// sol = nassign(node, depot_num, maxload, routes);
	depotnum = depot_num;
	vehicles = routes;
	customers = std::move(customer);
	depots = std::move(depot);
	nodes = node;
	initSol = nassign(customers, depots, maxload, routes, ctrl);
	initSol.update_hash(1);
	sol = initSol;
	bestSol.allobj = 100000000.0;
	bestSol.allength = 100000000.0;
}

void HLSVND::reset() {
	sol = initSol;
	bestSol.allobj = 100000000.0;
	bestSol.alltardiness = 100000000.0;
	bestSol.allength = 100000000.0;
	bestSol.shash.clear();
	bestSol.solution.clear();
	hists.clear();
	bestSol.valid = 0;
	info.one = 0;
	info.opt2 = 0;
	info.or2 = 0;
	info.or3 = 0;
	info.three = 0;
	info.or4 = 0;
	info.two = 0;
	info.arc = 0;
}

void HLSVND::run() {
	// u32 customer = nodes.size() - depotnum;
	//  sol.show();
	bool improved{1}, flag{0};
	int max_epoch{20};
	int epoch{max_epoch};
	float size_near{0.5}, T{1.0}, cold_rate{0.94};
	int vns[7] = {1, 2, 3, 4, 5, 6, 7};
	u32 maxcustomers = customers.size() * 10 > 100 ? customers.size() * 10 : 100;
	u32 stop{maxcustomers}, timelimit{0};
	Solution lsbest = bestSol;
	std::random_device rd;
	Xoshiro::Xoshiro128ss gen(rd());
	std::uniform_real_distribution<> dis(0, 1), dis2(0.2, 1);
	//  sol.evaluate();
	//  hist.emplace_back(sol.allobj);
	while (stop-- && improved) {
		improved = false;
		LS::relocate(sol, info.one, size_near, improved);
		LS::oropt2(sol, info.or2, size_near, improved);
		LS::arcnode(sol, info.three, size_near, improved);
		LS::oropt3(sol, info.or3, size_near, improved);
		LS::oropt4(sol, info.or4, size_near, improved);
		LS::arcswap(sol, info.arc, improved);
	}
	stop = maxcustomers;
#ifdef PR
	sol.alltardiness = priority(sol);
#endif
	sol.update();
	if (sol.valid)
		bestSol = lsbest = sol;
	while (epoch) {
		float r = dis2(gen);
		if (dis(gen) < 0.5) {
		    // PER::EjecChain(sol, vehicles * (1 - T) > 2 ? vehicles * (1 - T) : 2, 10 * (1 - T) > 1 ? 10 * (1 - T) : 1, 0);
		    // PER::EjecChain(sol, vehicles * T > 2 ? vehicles * T : 2, 10 * T > 1 ? 10 * T : 1, 0);
		    PER::EjecChain(sol, vehicles * r > 2 ? vehicles * r : 2, 10 * r > 1 ? 10 * r : 1, 0);
		    // PER::EjecChain(sol, vehicles * r, 10 * r, 0);
		} else {
		    // PER::RuinCreate(sol, (1 - T) / 2 > 0.1 ? (1 - T) / 2 : 0.1, customers, 10, 2);
		    // PER::RuinCreate(sol, T > 0.2 ? T / 2 : 0.1, customers, 10, 2);
		    PER::RuinCreate(sol, r > 0.2 ? r / 2 : 0.1, customers, 10, 2);
		    // PER::RuinCreate(sol, r / 2, customers, 10, 2);
		}
		for (u32 n{0}; n < 7;) {
			flag = 0;
			if (timelimit > maxcustomers) break;
			switch (vns[n]) {
			case 1:
				while (stop--) {
					LS::exchange(sol, info.two, size_near, improved);
					if (improved) {
						LS::twoopt(sol, info.opt2, improved);
						improved = 1;
						flag = 1;
					} else {
						break;
					}
				}
				break;
			case 2:
				while (stop--) {
					LS::relocate(sol, info.one, size_near, improved);
					if (improved) {
						LS::twoopt(sol, info.opt2, improved);
						improved = 1;
						flag = 1;
					} else {
						break;
					}
				}
				break;
			case 3:
				while (stop--) {
					LS::arcnode(sol, info.three, size_near, improved);
					if (improved) {
						LS::twoopt(sol, info.opt2, improved);
						improved = 1;
						flag = 1;
					} else {
						break;
					}
				}
				break;
			case 4:
				while (stop--) {
					LS::arcswap(sol, info.arc, improved);
					if (improved) {
						LS::twoopt(sol, info.opt2, improved);
						improved = 1;
						flag = 1;
					} else {
						break;
					}
				}
				break;
			case 5:
				while (stop--) {
					LS::oropt2(sol, info.or2, size_near, improved);
					if (improved) {
						LS::twoopt(sol, info.opt2, improved);
						improved = 1;
						flag = 1;
					} else {
						break;
					}
				}
				break;
			case 6:
				while (stop--) {
					LS::oropt3(sol, info.or3, size_near, improved);
					if (improved) {
						LS::twoopt(sol, info.opt2, improved);
						improved = 1;
						flag = 1;
					} else {
						break;
					}
				}
				break;
			case 7:
				while (stop--) {
					LS::oropt4(sol, info.or4, size_near, improved);
					if (improved) {
						LS::twoopt(sol, info.opt2, improved);
						improved = 1;
						flag = 1;
					} else {
						break;
					}
				}
				break;
			}
			timelimit++;
			if (stop == 0) flag = 0;
			stop = maxcustomers;
			if (flag)
				n = 0;
			else
				n++;
		}
		timelimit = 0;
		stop = maxcustomers;
		// sol.update();
		if (sol.valid) {
			if (sol.allobj < bestSol.allobj) {
				bestSol = sol;
			}
			if (sol.allobj < lsbest.allobj) {
				lsbest = sol;
				epoch = max_epoch;
			} else if (dis(gen) < T) {
				lsbest = sol;
				epoch = max_epoch;
			} else {
				sol = lsbest;
				epoch--;
			}
		} else {
			epoch--;
		}
		T *= cold_rate;
		hists.emplace_back(lsbest.allobj);
	}
	if (bestSol.allobj > lsbest.allobj || !bestSol.valid) {
		hists.emplace_back(lsbest.allobj);
		bestSol = std::move(lsbest);
	}
}

void SA::init(std::vector<Node*>& node, std::vector<Node*>& depot, std::vector<Node*>& customer, const u32 depot_num, u32 maxload, u32 routes) {
	depotnum = depot_num;
	vehicles = routes;
	customers = std::move(customer);
	depots = std::move(depot);
	nodes = node;
	initSol = assign(customers, depots, maxload, routes, ctrl);
	initSol.update_hash(1);
	sol = initSol;
	bestSol.allobj = 100000000.0;
}

void SA::reset() {
	sol = initSol;
	bestSol.allobj = 100000000.0;
	bestSol.alltardiness = 100000000.0;
	bestSol.allength = 100000000.0;
	bestSol.shash.clear();
	bestSol.solution.clear();
	bestSol.valid = 0;
	info.one = 0;
	info.opt2 = 0;
	info.or2 = 0;
	info.or3 = 0;
	info.three = 0;
	info.or4 = 0;
	info.two = 0;
	info.arc = 0;
}

void SA::run() {
	bool improved{1};
	int max_epoch{20};
	int epoch{max_epoch};
	float size_near{0.5}, T{1.0}, cold_rate{0.94};
	u32 maxcustomers = customers.size();
	u32 stop{maxcustomers};
	Solution lsbest = bestSol;
	std::random_device rd;
	Xoshiro::Xoshiro128ss gen(rd());
	std::uniform_real_distribution<> dis(0, 1);
	// sol.evaluate();
	while (stop-- && improved) {
		improved = false;
		LS::relocate(sol, info.one, size_near, improved);
		LS::oropt2(sol, info.or2, size_near, improved);
		LS::arcnode(sol, info.three, size_near, improved);
		LS::oropt3(sol, info.or3, size_near, improved);
		LS::oropt4(sol, info.or4, size_near, improved);
		LS::arcswap(sol, info.arc, improved);
	}
	stop = maxcustomers * 10;
	sol.alltardiness = priority(sol);
	sol.update();
	// sol.show();
	if (sol.valid)
		bestSol = lsbest = sol;
	while (epoch) {
		// std::shuffle(vns, vns + 7, gen);
		if (dis(gen) < 0.5) {
			u32 k = vehicles / 2;
			if (k < 2)
				k = 2;
			else if (k > 4)
				k = 4;
			PER::EjecChain(sol, k, 50);
			// PER::EjecChain(sol, vehicles * T > 2 ? vehicles * T : 2, 10 * T > 1 ? 10 * T : 1, 0);
			// PER::EjecChain(sol, vehicles * r > 2 ? vehicles * r : 2, 10 * r > 1 ? 10 * r : 1, 0);
		} else {
			PER::RuinCreate(sol, 0.3, customers, 10);
			// PER::RuinCreate(sol, T > 0.2 ? T / 2 : 0.1, customers, 10, 2);
			// PER::RuinCreate(sol, r > 0.2 ? r / 2 : 0.1, customers, 10, 2);
		}
		//}
		improved = 1;
		while (improved) {
			improved = false;
			LS::relocate(sol, info.one, size_near, improved);
		}
		improved = 1;
		while (improved) {
			improved = false;
			LS::oropt2(sol, info.or2, size_near, improved);
		}
		improved = 1;
		while (improved) {
			improved = false;
			LS::arcnode(sol, info.three, size_near, improved);
		}
		improved = 1;
		while (improved) {
			improved = false;
			LS::oropt3(sol, info.or3, size_near, improved);
		}
		improved = 1;
		while (improved) {
			improved = false;
			LS::oropt4(sol, info.or4, size_near, improved);
		}
		improved = 1;
		while (improved) {
			improved = false;
			LS::arcswap(sol, info.arc, improved);
		}
		improved = 1;
		while (improved) {
			improved = false;
			LS::exchange(sol, info.two, size_near, improved);
		}
		improved = 1;
		while (improved) {
			improved = false;
			LS::twoopt(sol, info.opt2, improved);
		}
		improved = 1;
		// sol.update();
		if (sol.valid) {
			if (sol.allobj < bestSol.allobj) {
				bestSol = sol;
			}
			// if (T < 0.2 && change) {
			//	if (bestSol.valid)
			//		lsbest = bestSol;
			//	change = 0;
			// }
			if (sol.allobj < lsbest.allobj) {
				lsbest = sol;
				epoch = max_epoch;
			} else if (dis(gen) < T) {
				lsbest = sol;
				epoch = max_epoch;
			} else {
				sol = lsbest;
				epoch--;
			}
			T *= cold_rate;
		}
		stop--;
		if (stop == 0) break;
	}
	if (bestSol.allobj > lsbest.allobj || !bestSol.valid) {
		bestSol = std::move(lsbest);
	}
}

void VNS::init(std::vector<Node*>& node, std::vector<Node*>& depot, std::vector<Node*>& customer, const u32 depot_num, u32 maxload, u32 routes) {
	depotnum = depot_num;
	vehicles = routes;
	customers = std::move(customer);
	depots = std::move(depot);
	nodes = node;
	initSol = assign(customers, depots, maxload, routes, ctrl);
	initSol.update_hash(1);
	sol = initSol;
	bestSol.allobj = 100000000.0;
}

void VNS::reset() {
	sol = initSol;
	bestSol.allobj = 100000000.0;
	bestSol.alltardiness = 100000000.0;
	bestSol.allength = 100000000.0;
	bestSol.shash.clear();
	bestSol.solution.clear();
	bestSol.valid = 0;
	info.one = 0;
	info.opt2 = 0;
	info.or2 = 0;
	info.or3 = 0;
	info.three = 0;
	info.or4 = 0;
	info.two = 0;
	info.arc = 0;
}

void VNS::run() {
	bool improved{1};
	int max_epoch{20};
	int epoch{max_epoch};
	float size_near{0.5};
	u32 maxcustomers = customers.size();
	u32 stop{maxcustomers};
	int vns[4] = {1, 2, 3, 4};
	Solution lsbest = bestSol;
	std::random_device rd;
	Xoshiro::Xoshiro128ss gen(rd());
	std::uniform_real_distribution<> dis(0, 1);
	// sol.evaluate();
	while (stop-- && improved) {
		improved = false;
		LS::relocate(sol, info.one, size_near, improved);
		LS::oropt2(sol, info.or2, size_near, improved);
		LS::arcnode(sol, info.three, size_near, improved);
		LS::oropt3(sol, info.or3, size_near, improved);
		LS::oropt4(sol, info.or4, size_near, improved);
		LS::arcswap(sol, info.arc, improved);
	}
	stop = maxcustomers * 10;
	sol.alltardiness = priority(sol);
	sol.update();
	// sol.show();
	if (sol.valid)
		bestSol = lsbest = sol;
	while (epoch) {
		// std::shuffle(vns, vns + 7, gen);
		for (u32 i{0}; i < 4;) {
			if (vns[i] == 1) {
				SHACK::twoopt(sol, 0.1, 50);
			} else if (vns[i] == 2) {
				SHACK::oropt(sol, 0.1, 50);
			} else if (vns[i] == 3) {
				SHACK::arcnode(sol, 0.1, 50);
			} else if (vns[i] == 4) {
				SHACK::arcswap(sol, 0.1, 50);
			}
			improved = 1;
			while (improved) {
				improved = false;
				LS::relocate(sol, info.one, size_near, improved);
				LS::exchange(sol, info.two, size_near, improved);
				LS::oropt2(sol, info.or2, size_near, improved);
				LS::arcnode(sol, info.three, size_near, improved);
				LS::oropt3(sol, info.or3, size_near, improved);
				LS::oropt4(sol, info.or4, size_near, improved);
				LS::arcswap(sol, info.arc, improved);
				LS::twoopt(sol, info.opt2, improved);
				// if (improved) change = 1;
				stop--;
				if (stop == 0) break;
			}
			if (sol.valid) {
				if (sol.allobj < lsbest.allobj) {
					lsbest = sol;
					i = 0;
				} else {
					sol = lsbest;
					i++;
				}
			} else {
				lsbest = sol;
				i++;
			}
			stop = maxcustomers * 10;
		}
		if (lsbest.valid) {
			if (lsbest.allobj < bestSol.allobj) {
				bestSol = lsbest;
				epoch = max_epoch;
			} else {
				lsbest = bestSol;
				epoch--;
			}
		} else {
			lsbest = bestSol;
			epoch--;
		}
	}
}

