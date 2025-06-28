#pragma once

#include "core/external/TestWrapper.h"
#include "core/logging/Logger.h"
#include <chrono>
#include <vector>
#include <numeric>
#include <algorithm>
#include <iomanip>
#include <fstream>
#include <sstream>

namespace iloss {
namespace test {

/**
 * @brief Performance measurement result
 */
struct BenchmarkResult {
    std::string name;
    size_t iterations;
    double totalTime;        // milliseconds
    double averageTime;      // milliseconds
    double minTime;          // milliseconds
    double maxTime;          // milliseconds
    double standardDeviation;// milliseconds
    double opsPerSecond;     // operations per second
    std::vector<double> samples; // individual timings
};

/**
 * @brief Base class for performance benchmarks
 */
class BenchmarkBase : public ::testing::Test {
public:
    struct Config {
        size_t warmupIterations = 10;
        size_t minIterations = 100;
        size_t maxIterations = 10000;
        double minDuration = 1000.0;  // milliseconds
        bool collectSamples = true;
        bool verbose = false;
        std::string outputFile;
    };
    
protected:
    void SetUp() override {
        m_results.clear();
    }
    
    void TearDown() override {
        if (!m_config.outputFile.empty()) {
            exportResults(m_config.outputFile);
        }
        
        if (m_config.verbose) {
            printResults();
        }
    }
    
    /**
     * @brief Run a benchmark with automatic iteration count determination
     */
    template<typename Func>
    BenchmarkResult benchmark(const std::string& name, Func&& func) {
        // Warmup
        for (size_t i = 0; i < m_config.warmupIterations; ++i) {
            func();
        }
        
        // Determine iteration count
        size_t iterations = determineIterationCount(func);
        
        // Run benchmark
        return runBenchmark(name, func, iterations);
    }
    
    /**
     * @brief Run a benchmark with fixed iteration count
     */
    template<typename Func>
    BenchmarkResult benchmarkFixed(const std::string& name, Func&& func, size_t iterations) {
        // Warmup
        for (size_t i = 0; i < m_config.warmupIterations; ++i) {
            func();
        }
        
        return runBenchmark(name, func, iterations);
    }
    
    /**
     * @brief Compare two implementations
     */
    template<typename Func1, typename Func2>
    void compareBenchmarks(const std::string& name1, Func1&& func1,
                          const std::string& name2, Func2&& func2) {
        auto result1 = benchmark(name1, func1);
        auto result2 = benchmark(name2, func2);
        
        double speedup = result1.averageTime / result2.averageTime;
        std::string faster = speedup > 1.0 ? name2 : name1;
        double factor = speedup > 1.0 ? speedup : 1.0 / speedup;
        
        LOG_INFO("Comparison: {} is {:.2f}x faster than {}", 
                faster, factor, speedup > 1.0 ? name1 : name2);
    }
    
    /**
     * @brief Run a benchmark with varying input sizes
     */
    template<typename Func>
    void benchmarkScaling(const std::string& baseName, 
                         Func&& func,
                         const std::vector<size_t>& sizes) {
        for (size_t size : sizes) {
            std::string name = baseName + "_" + std::to_string(size);
            auto result = benchmark(name, [&]() { func(size); });
            
            // Calculate complexity (assuming linear for now)
            double timePerElement = result.averageTime / size;
            LOG_INFO("{}: {:.6f} ms/element", name, timePerElement);
        }
    }
    
    // Result access
    const std::vector<BenchmarkResult>& getResults() const { return m_results; }
    
    BenchmarkResult* findResult(const std::string& name) {
        auto it = std::find_if(m_results.begin(), m_results.end(),
            [&name](const BenchmarkResult& r) { return r.name == name; });
        return it != m_results.end() ? &(*it) : nullptr;
    }
    
    // Configuration
    void setConfig(const Config& config) { m_config = config; }
    Config& getConfig() { return m_config; }
    
protected:
    /**
     * @brief Time a single execution
     */
    template<typename Func>
    double timeExecution(Func&& func) {
        auto start = std::chrono::high_resolution_clock::now();
        func();
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
        return duration.count() / 1e6;  // Convert to milliseconds
    }
    
    /**
     * @brief Determine optimal iteration count
     */
    template<typename Func>
    size_t determineIterationCount(Func&& func) {
        // Quick timing to estimate
        double singleTime = timeExecution(func);
        
        if (singleTime <= 0.0) {
            return m_config.maxIterations;
        }
        
        // Calculate iterations needed for minimum duration
        size_t iterations = static_cast<size_t>(m_config.minDuration / singleTime);
        
        // Clamp to configured range
        iterations = std::max(iterations, m_config.minIterations);
        iterations = std::min(iterations, m_config.maxIterations);
        
        return iterations;
    }
    
    /**
     * @brief Run the actual benchmark
     */
    template<typename Func>
    BenchmarkResult runBenchmark(const std::string& name, Func&& func, size_t iterations) {
        BenchmarkResult result;
        result.name = name;
        result.iterations = iterations;
        
        std::vector<double> times;
        times.reserve(iterations);
        
        // Run iterations
        auto totalStart = std::chrono::high_resolution_clock::now();
        
        for (size_t i = 0; i < iterations; ++i) {
            double time = timeExecution(func);
            if (m_config.collectSamples) {
                times.push_back(time);
            }
        }
        
        auto totalEnd = std::chrono::high_resolution_clock::now();
        auto totalDuration = std::chrono::duration_cast<std::chrono::nanoseconds>(
            totalEnd - totalStart);
        result.totalTime = totalDuration.count() / 1e6;
        
        if (m_config.collectSamples && !times.empty()) {
            result.samples = times;
            result.averageTime = std::accumulate(times.begin(), times.end(), 0.0) / times.size();
            result.minTime = *std::min_element(times.begin(), times.end());
            result.maxTime = *std::max_element(times.begin(), times.end());
            
            // Calculate standard deviation
            double variance = 0.0;
            for (double time : times) {
                double diff = time - result.averageTime;
                variance += diff * diff;
            }
            variance /= times.size();
            result.standardDeviation = std::sqrt(variance);
        } else {
            result.averageTime = result.totalTime / iterations;
            result.minTime = result.averageTime;
            result.maxTime = result.averageTime;
            result.standardDeviation = 0.0;
        }
        
        result.opsPerSecond = 1000.0 / result.averageTime;  // Convert from ms to ops/sec
        
        m_results.push_back(result);
        return result;
    }
    
    /**
     * @brief Print benchmark results
     */
    void printResults() {
        std::cout << "\nBenchmark Results:\n";
        std::cout << std::string(80, '-') << "\n";
        std::cout << std::left << std::setw(30) << "Name"
                  << std::right << std::setw(10) << "Iterations"
                  << std::setw(12) << "Avg (ms)"
                  << std::setw(12) << "Min (ms)"
                  << std::setw(12) << "Max (ms)"
                  << std::setw(12) << "StdDev"
                  << std::setw(12) << "Ops/sec"
                  << "\n";
        std::cout << std::string(80, '-') << "\n";
        
        for (const auto& result : m_results) {
            std::cout << std::left << std::setw(30) << result.name
                      << std::right << std::setw(10) << result.iterations
                      << std::setw(12) << std::fixed << std::setprecision(6) << result.averageTime
                      << std::setw(12) << result.minTime
                      << std::setw(12) << result.maxTime
                      << std::setw(12) << std::setprecision(3) << result.standardDeviation
                      << std::setw(12) << std::setprecision(0) << result.opsPerSecond
                      << "\n";
        }
        std::cout << std::string(80, '-') << "\n";
    }
    
    /**
     * @brief Export results to file
     */
    void exportResults(const std::string& filename) {
        if (filename.ends_with(".csv")) {
            exportCSV(filename);
        } else if (filename.ends_with(".json")) {
            exportJSON(filename);
        } else {
            exportText(filename);
        }
    }
    
    void exportCSV(const std::string& filename) {
        std::ofstream file(filename);
        if (!file.is_open()) return;
        
        file << "Name,Iterations,Total(ms),Average(ms),Min(ms),Max(ms),StdDev,Ops/sec\n";
        
        for (const auto& result : m_results) {
            file << result.name << ","
                 << result.iterations << ","
                 << result.totalTime << ","
                 << result.averageTime << ","
                 << result.minTime << ","
                 << result.maxTime << ","
                 << result.standardDeviation << ","
                 << result.opsPerSecond << "\n";
        }
    }
    
    void exportJSON(const std::string& filename) {
        nlohmann::json root = nlohmann::json::array();
        
        for (const auto& result : m_results) {
            nlohmann::json item;
            item["name"] = result.name;
            item["iterations"] = result.iterations;
            item["totalTime"] = result.totalTime;
            item["averageTime"] = result.averageTime;
            item["minTime"] = result.minTime;
            item["maxTime"] = result.maxTime;
            item["standardDeviation"] = result.standardDeviation;
            item["opsPerSecond"] = result.opsPerSecond;
            
            if (!result.samples.empty()) {
                item["samples"] = result.samples;
            }
            
            root.push_back(item);
        }
        
        std::ofstream file(filename);
        if (file.is_open()) {
            file << root.dump(4);
        }
    }
    
    void exportText(const std::string& filename) {
        std::ofstream file(filename);
        if (!file.is_open()) return;
        
        file << "ILOSS Benchmark Results\n";
        file << "=====================\n\n";
        
        for (const auto& result : m_results) {
            file << "Benchmark: " << result.name << "\n";
            file << "  Iterations:    " << result.iterations << "\n";
            file << "  Total time:    " << result.totalTime << " ms\n";
            file << "  Average time:  " << result.averageTime << " ms\n";
            file << "  Min time:      " << result.minTime << " ms\n";
            file << "  Max time:      " << result.maxTime << " ms\n";
            file << "  Std deviation: " << result.standardDeviation << " ms\n";
            file << "  Throughput:    " << result.opsPerSecond << " ops/sec\n";
            file << "\n";
        }
    }
    
private:
    Config m_config;
    std::vector<BenchmarkResult> m_results;
};

/**
 * @brief Macro for defining benchmark tests
 */
#define BENCHMARK_F(test_fixture, test_name) \
    TEST_F(test_fixture, test_name)

/**
 * @brief Macro for simple benchmark timing
 */
#define BENCHMARK_TIME(name, code) \
    do { \
        auto result = benchmark(name, [&]() { code; }); \
        EXPECT_GT(result.opsPerSecond, 0); \
    } while(0)

} // namespace test
} // namespace iloss