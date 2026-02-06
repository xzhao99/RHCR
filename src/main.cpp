#include "KivaSystem.h"
#include "SortingSystem.h"
#include "OnlineSystem.h"
#include "BeeSystem.h"
#include "ID.h"
#include <boost/program_options.hpp>
#include <cassert>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>


void set_parameters(BasicSystem& system, const boost::program_options::variables_map& vm)
{
	system.outfile = vm["output"].as<std::string>();
	system.screen = vm["screen"].as<int>();
	system.log = vm["log"].as<bool>();
	system.num_of_drives = vm["agentNum"].as<int>();
	system.time_limit = vm["cutoffTime"].as<int>();
	system.simulation_window = vm["simulation_window"].as<int>();
	system.planning_window = vm["planning_window"].as<int>();
	system.travel_time_window = vm["travel_time_window"].as<int>();
	system.consider_rotation = vm["rotation"].as<bool>();
	system.k_robust = vm["robust"].as<int>();
	system.hold_endpoints = vm["hold_endpoints"].as<bool>();
	system.useDummyPaths = vm["dummy_paths"].as<bool>();
	if (vm.count("seed"))
		system.seed = vm["seed"].as<int>();
	else
		system.seed = (int)time(0);
	srand(system.seed);
}


struct SolverBundle
{
	std::unique_ptr<SingleAgentSolver> path_planner;
	std::unique_ptr<MAPFSolver> base_solver;
	std::unique_ptr<MAPFSolver> solver;
};

SolverBundle make_solvers(const BasicGraph& G, const boost::program_options::variables_map& vm)
{
	SolverBundle bundle;
	std::string solver_name = vm["single_agent_solver"].as<std::string>();
	if (solver_name == "ASTAR")
	{
		bundle.path_planner = std::make_unique<StateTimeAStar>();
	}
	else if (solver_name == "SIPP")
	{
		bundle.path_planner = std::make_unique<SIPP>();
	}
	else
	{
		std::cout << "Single-agent solver " << solver_name << "does not exist!" << std::endl;
		exit(-1);
	}

	solver_name = vm["solver"].as<std::string>();
	if (solver_name == "ECBS")
	{
		auto ecbs = std::make_unique<ECBS>(G, *bundle.path_planner);
		ecbs->potential_function = vm["potential_function"].as<std::string>();
		ecbs->potential_threshold = vm["potential_threshold"].as<double>();
		ecbs->suboptimal_bound = vm["suboptimal_bound"].as<double>();
		bundle.base_solver = std::move(ecbs);
	}
	else if (solver_name == "PBS")
	{
		auto pbs = std::make_unique<PBS>(G, *bundle.path_planner);
		pbs->lazyPriority = vm["lazyP"].as<bool>();
        auto prioritize_start = vm["prioritize_start"].as<bool>();
        if (vm["hold_endpoints"].as<bool>() or vm["dummy_paths"].as<bool>())
            prioritize_start = false;
        pbs->prioritize_start = prioritize_start;
        pbs->setRT(vm["CAT"].as<bool>(), prioritize_start);
		bundle.base_solver = std::move(pbs);
	}
	else if (solver_name == "WHCA")
	{
		bundle.base_solver = std::make_unique<WHCAStar>(G, *bundle.path_planner);
	}
	else if (solver_name == "LRA")
	{
		bundle.base_solver = std::make_unique<LRAStar>(G, *bundle.path_planner);
	}
	else
	{
		std::cout << "Solver " << solver_name << "does not exist!" << std::endl;
		exit(-1);
	}

	if (vm["id"].as<bool>())
	{
		bundle.solver = std::make_unique<ID>(G, *bundle.path_planner, *bundle.base_solver);
	}
	else
	{
		bundle.solver = std::move(bundle.base_solver);
	}
	return bundle;
}

class ScenarioRunner
{
public:
	virtual ~ScenarioRunner() = default;
	virtual int run() = 0;
};

class KivaScenario final : public ScenarioRunner
{
public:
	static std::unique_ptr<KivaScenario> create(const boost::program_options::variables_map& vm)
	{
		auto scenario = std::unique_ptr<KivaScenario>(new KivaScenario(vm));
		if (!scenario->G.load_map(vm["map"].as<std::string>()))
		{
			return nullptr;
		}
		scenario->solvers = make_solvers(scenario->G, vm);
		scenario->system = std::make_unique<KivaSystem>(scenario->G, *scenario->solvers.solver);
		return scenario;
	}

	int run() override
	{
		set_parameters(*system, vm);
		G.preprocessing(system->consider_rotation);
		system->simulate(vm["simulation_time"].as<int>());
		return 0;
	}

private:
	explicit KivaScenario(const boost::program_options::variables_map& vm) : vm(vm) {}

	const boost::program_options::variables_map& vm;
	KivaGrid G;
	SolverBundle solvers;
	std::unique_ptr<KivaSystem> system;
};

class SortingScenario final : public ScenarioRunner
{
public:
	static std::unique_ptr<SortingScenario> create(const boost::program_options::variables_map& vm)
	{
		auto scenario = std::unique_ptr<SortingScenario>(new SortingScenario(vm));
		if (!scenario->G.load_map(vm["map"].as<std::string>()))
		{
			return nullptr;
		}
		scenario->solvers = make_solvers(scenario->G, vm);
		scenario->system = std::make_unique<SortingSystem>(scenario->G, *scenario->solvers.solver);
		return scenario;
	}

	int run() override
	{
		assert(!system->hold_endpoints);
		assert(!system->useDummyPaths);
		set_parameters(*system, vm);
		G.preprocessing(system->consider_rotation);
		system->simulate(vm["simulation_time"].as<int>());
		return 0;
	}

private:
	explicit SortingScenario(const boost::program_options::variables_map& vm) : vm(vm) {}

	const boost::program_options::variables_map& vm;
	SortingGrid G;
	SolverBundle solvers;
	std::unique_ptr<SortingSystem> system;
};

class OnlineScenario final : public ScenarioRunner
{
public:
	static std::unique_ptr<OnlineScenario> create(const boost::program_options::variables_map& vm)
	{
		auto scenario = std::unique_ptr<OnlineScenario>(new OnlineScenario(vm));
		if (!scenario->G.load_map(vm["map"].as<std::string>()))
		{
			return nullptr;
		}
		scenario->solvers = make_solvers(scenario->G, vm);
		scenario->system = std::make_unique<OnlineSystem>(scenario->G, *scenario->solvers.solver);
		return scenario;
	}

	int run() override
	{
		assert(!system->hold_endpoints);
		assert(!system->useDummyPaths);
		set_parameters(*system, vm);
		G.preprocessing(system->consider_rotation);
		system->simulate(vm["simulation_time"].as<int>());
		return 0;
	}

private:
	explicit OnlineScenario(const boost::program_options::variables_map& vm) : vm(vm) {}

	const boost::program_options::variables_map& vm;
	OnlineGrid G;
	SolverBundle solvers;
	std::unique_ptr<OnlineSystem> system;
};

class BeeScenario final : public ScenarioRunner
{
public:
	static std::unique_ptr<BeeScenario> create(const boost::program_options::variables_map& vm, clock_t start_time)
	{
		auto scenario = std::unique_ptr<BeeScenario>(new BeeScenario(vm, start_time));
		if (!scenario->G.load_map(vm["map"].as<std::string>()))
		{
			return nullptr;
		}
		scenario->solvers = make_solvers(scenario->G, vm);
		scenario->system = std::make_unique<BeeSystem>(scenario->G, *scenario->solvers.solver);
		return scenario;
	}

	int run() override
	{
		assert(!system->hold_endpoints);
		assert(!system->useDummyPaths);
		set_parameters(*system, vm);
		G.preprocessing(vm["task"].as<std::string>(), system->consider_rotation);
		system->load_task_assignments(vm["task"].as<std::string>());
		system->simulate();
		double runtime = (double)(clock() - start_time) / CLOCKS_PER_SEC;
		std::cout << "Overall runtime:			" << runtime << " seconds." << std::endl;
		// std::cout << "	Reading from file:		" << G.loading_time + system->loading_time << " seconds." << std::endl;
		// std::cout << "	Preprocessing:			" << G.preprocessing_time << " seconds." << std::endl;
		// std::cout << "	Writing to file:		" << system->saving_time << " seconds." << std::endl;
		std::cout << "Makespan:		" << system->get_makespan() << " timesteps." << std::endl;
		std::cout << "Flowtime:		" << system->get_flowtime() << " timesteps." << std::endl;
		std::cout << "Flowtime lowerbound:	" << system->get_flowtime_lowerbound() << " timesteps." << std::endl;
		auto flower_ids = system->get_missed_flower_ids();
		std::cout << "Missed tasks:";
		for (auto id : flower_ids)
			std::cout << " " << id;
		std::cout << std::endl;
		// std::cout << "Remaining tasks: " << system->get_num_of_remaining_tasks() << std::endl;
		std::cout << "Objective: " << system->get_objective() << std::endl;
		std::ofstream output;
		output.open(vm["output"].as<std::string>() + "/MAPF_results.txt", std::ios::out);
		output << "Overall runtime: " << runtime << " seconds." << std::endl;
		output << "Makespan: " << system->get_makespan() << " timesteps." << std::endl;
		output << "Flowtime: " << system->get_flowtime() << " timesteps." << std::endl;
		output << "Flowtime lowerbound: " << system->get_flowtime_lowerbound() << " timesteps." << std::endl;
		output << "Missed tasks:";
		for (auto id : flower_ids)
			output << " " << id;
		output << std::endl;
		output << "Objective: " << system->get_objective() << std::endl;
		output.close();
		return 0;
	}

private:
	BeeScenario(const boost::program_options::variables_map& vm, clock_t start_time)
		: vm(vm), start_time(start_time) {}

	const boost::program_options::variables_map& vm;
	clock_t start_time;
	BeeGraph G;
	SolverBundle solvers;
	std::unique_ptr<BeeSystem> system;
};

std::unique_ptr<ScenarioRunner> make_scenario(const boost::program_options::variables_map& vm, clock_t start_time, int& status)
{
	status = 0;
	const std::string scenario_name = vm["scenario"].as<std::string>();
	if (scenario_name == "KIVA")
	{
		auto scenario = KivaScenario::create(vm);
		if (!scenario)
		{
			status = -1;
			return nullptr;
		}
		return scenario;
	}
	if (scenario_name == "SORTING")
	{
		auto scenario = SortingScenario::create(vm);
		if (!scenario)
		{
			status = -1;
			return nullptr;
		}
		return scenario;
	}
	if (scenario_name == "ONLINE")
	{
		auto scenario = OnlineScenario::create(vm);
		if (!scenario)
		{
			status = -1;
			return nullptr;
		}
		return scenario;
	}
	if (scenario_name == "BEE")
	{
		auto scenario = BeeScenario::create(vm, start_time);
		if (!scenario)
		{
			status = -1;
			return nullptr;
		}
		return scenario;
	}
	std::cout << "Scenario " << scenario_name << "does not exist!" << std::endl;
	status = -1;
	return nullptr;
}


int main(int argc, char** argv) 
{
	namespace po = boost::program_options;
	// Declare the supported options.
	po::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")
		("scenario", po::value<std::string>()->required(), "scenario (SORTING, KIVA, ONLINE, BEE)")
		("map,m", po::value<std::string>()->required(), "input map file")
		("task", po::value<std::string>()->default_value(""), "input task file")
		("output,o", po::value<std::string>()->default_value("../exp/test"), "output folder name")
		("agentNum,k", po::value<int>()->required(), "number of drives")
		("cutoffTime,t", po::value<int>()->default_value(60), "cutoff time (seconds)")
		("seed,d", po::value<int>(), "random seed")
		("screen,s", po::value<int>()->default_value(1), "screen option (0: none; 1: results; 2:all)")
		("solver", po::value<std::string>()->default_value("PBS"), "solver (LRA, PBS, WHCA, ECBS)")
		("id", po::value<bool>()->default_value(false), "independence detection")
		("single_agent_solver", po::value<std::string>()->default_value("SIPP"), "single-agent solver (ASTAR, SIPP)")
		("lazyP", po::value<bool>()->default_value(false), "use lazy priority")
		("simulation_time", po::value<int>()->default_value(5000), "run simulation")
		("simulation_window", po::value<int>()->default_value(5), "call the planner every simulation_window timesteps")
		("travel_time_window", po::value<int>()->default_value(0), "consider the traffic jams within the given window")
		("planning_window", po::value<int>()->default_value(INT_MAX / 2),
		        "the planner outputs plans with first planning_window timesteps collision-free")
		("potential_function", po::value<std::string>()->default_value("NONE"), "potential function (NONE, SOC, IC)")
		("potential_threshold", po::value<double>()->default_value(0), "potential threshold")
		("rotation", po::value<bool>()->default_value(false), "consider rotation")
		("robust", po::value<int>()->default_value(0), "k-robust (for now, only work for PBS)")
		("CAT", po::value<bool>()->default_value(false), "use conflict-avoidance table")
		// ("PG", po::value<bool>()->default_value(false),
		//        "reuse the priority graph of the goal node of the previous search")
		("hold_endpoints", po::value<bool>()->default_value(false),
		        "Hold endpoints from Ma et al, AAMAS 2017")
		("dummy_paths", po::value<bool>()->default_value(false),
				"Find dummy paths from Liu et al, AAMAS 2019")
		("prioritize_start", po::value<bool>()->default_value(true), "Prioritize waiting at start locations")
		("suboptimal_bound", po::value<double>()->default_value(1), "Suboptimal bound for ECBS")
		("log", po::value<bool>()->default_value(false), "save the search trees (and the priority trees)")
		;
	clock_t start_time = clock();
	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);

	if (vm.count("help")) {
		std::cout << desc << std::endl;
		return 1;
	}

	po::notify(vm);

    // check params
    if (vm["hold_endpoints"].as<bool>() or vm["dummy_paths"].as<bool>())
    {
        if (vm["hold_endpoints"].as<bool>() and vm["dummy_paths"].as<bool>())
        {
            std::cerr << "Hold endpoints and dummy paths cannot be used simultaneously" << std::endl;
            exit(-1);
        }
        if (vm["simulation_window"].as<int>() != 1)
        {
            std::cerr << "Hold endpoints and dummy paths can only work when the simulation window is 1" << std::endl;
            exit(-1);
        }
        if (vm["planning_window"].as<int>() < INT_MAX / 2)
        {
            std::cerr << "Hold endpoints and dummy paths cannot work with planning windows" << std::endl;
            exit(-1);
        }
    }

    // make dictionary
	std::filesystem::path dir(vm["output"].as<std::string>() +"/");
	std::filesystem::create_directories(dir);
	if (vm["log"].as<bool>())
	{
		std::filesystem::path dir1(vm["output"].as<std::string>() + "/goal_nodes/");
		std::filesystem::path dir2(vm["output"].as<std::string>() + "/search_trees/");
		std::filesystem::create_directories(dir1);
		std::filesystem::create_directories(dir2);
	}


	int status = 0;
	auto scenario = make_scenario(vm, start_time, status);
	if (!scenario)
	{
		return status;
	}
	return scenario->run();
}
