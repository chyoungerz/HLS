// #include <ctime>
#include <chrono>
#include <iostream>
#include <vector>

#include "fileio.hpp"
#include "heuristic.hpp"
#include "node.hpp"

static const int _ = []() { std::ios::sync_with_stdio(false); std::cin.tie(nullptr); std::cout.tie(nullptr); return 0; }();

using namespace std;
/// @brief
/// @param argv 文件 结果文件 迭代次数 接受准则等
int main(int argc, char const *argv[]) {
	// ios::sync_with_stdio(false);
	//  const time_t now = time(nullptr);  // 以当前时间设置随机数种子
	string file, result, resulthistory;
	vector<Node *> nodes, depots, customers;
	u32 maxload{}, depot_num{}, routes{}, epoch{};
	Solution bestsol;
	bestsol.allobj = 1000000000;
	if (argc != 3) {
		file = "P-n16-k8-PrK.vrp";
		result = "data.txt";
		routes = 8;
		cerr << "no enought args" << endl;
		cerr << "use default: " << file << endl;
	} else {
		file = argv[1];
		if (file.size() < 10) {
			cerr << "file: " << file << " name error" << endl;
			return 1;
		}
		if (file[file.size() - 10] == 'k') {
			routes = atoi(file.substr(file.size() - 9, 1).c_str());
		} else {
			routes = atoi(file.substr(file.size() - 10, 2).c_str());
		}
		result = "results/" + file.substr(2, file.size() - 6) + ".txt";
		resulthistory = "results/info/" + file.substr(2, file.size() - 6) + ".log";
		epoch = atoi(argv[2]);
	}
	// nodes = read(file, maxload, depot_num, routes);
	read_vrp(file, maxload, depot_num, routes, nodes);
	if (nodes.empty()) {
		cerr << "file: " << file << " open failed" << endl;
		return 1;
	}
	init_distance(nodes, depot_num, depots, customers);  // 计算客户距离
	HLSVND vrp;
	vrp.init(nodes, depots, customers, depot_num, maxload, routes);
#ifdef NDEBUG
	Info infos;
	vector<double> lengths, objs, tardiness, history;
	lengths.reserve(10);
	objs.reserve(10);
	tardiness.reserve(10);
	auto t1{chrono::high_resolution_clock::now()};
	for (u32 i{0}; i < epoch; i++) {
		vrp.run();
		if (vrp.bestSol.valid) {
			lengths.emplace_back(vrp.bestSol.allength);
			objs.emplace_back(vrp.bestSol.allobj);
			tardiness.emplace_back(vrp.bestSol.alltardiness);
			infos = vrp.info;
			hist(resulthistory, vrp.bestSol, vrp.info);
			if (bestsol.allobj > vrp.bestSol.allobj) {
				bestsol = vrp.bestSol;
			}
		}
		// write("results/epoch.csv", vrp.hists);
		vrp.reset();
	}
	auto t2{chrono::high_resolution_clock::now()};
	u64 duration = (chrono::duration_cast<chrono::milliseconds>(t2 - t1)).count();
	write(result, bestsol, infos, lengths, objs, tardiness, duration, maxload);
	// hist(result, history);
	bool pass = true;
	for (auto i : bestsol.solution) {
		if (i.load > maxload) {
			pass = false;
			break;
		}
	}
	release(nodes);
	if (pass) {
		return 0;
	} else {
		return 1;
	}
#else
	vrp.run();
	vrp.bestSol.show();
	release(nodes);
	return 0;
#endif
}
