#pragma once

#include "../interfaces/ITractionControlModule.hpp"
#include "../interfaces/ITractionInterfaces.hpp"
#include <memory>
#include <vector>
#include <map>
#include <functional>

namespace traction_control {

/**
 * @brief Central coordinator for all traction control modules
 *
 * The TractionControlManager coordinates multiple traction control modules,
 * manages their lifecycle, handles data flow, and combines their outputs.
 */
class TractionControlManager {
public:
    TractionControlManager();
    ~TractionControlManager();

    /**
     * @brief Initialize the manager and all registered modules
     * @return true if initialization successful
     */
    bool initialize();

    /**
     * @brief Process traction control for current state
     * @param state Current traction state
     * @param command Output combined traction command
     * @return true if processing successful
     */
    bool process(const TractionState& state, TractionCommand& command);

    /**
     * @brief Register a traction control module
     * @param module Module to register
     * @return true if registration successful
     */
    bool registerModule(TractionControlModulePtr module);

    /**
     * @brief Unregister a traction control module
     * @param module_name Name of module to unregister
     * @return true if unregistration successful
     */
    bool unregisterModule(const char* module_name);

    /**
     * @brief Enable/disable a specific module
     * @param module_name Module name
     * @param enabled Enable state
     * @return true if operation successful
     */
    bool setModuleEnabled(const char* module_name, bool enabled);

    /**
     * @brief Get module by name
     * @param module_name Module name
     * @return Module pointer or nullptr if not found
     */
    TractionControlModulePtr getModule(const char* module_name) const;

    /**
     * @brief Get all registered modules
     * @return Vector of all modules
     */
    std::vector<TractionControlModulePtr> getAllModules() const;

    /**
     * @brief Register data provider
     * @param provider Data provider to register
     */
    void registerDataProvider(std::shared_ptr<ITractionDataProvider> provider);

    /**
     * @brief Register command executor
     * @param executor Command executor to register
     */
    void registerCommandExecutor(std::shared_ptr<ITractionCommandExecutor> executor);

    /**
     * @brief Register state observer
     * @param observer State observer to register
     */
    void registerStateObserver(std::shared_ptr<ITractionStateObserver> observer);

    /**
     * @brief Register command observer
     * @param observer Command observer to register
     */
    void registerCommandObserver(std::shared_ptr<ITractionCommandObserver> observer);

    /**
     * @brief Update parameters for all modules
     * @return true if all updates successful
     */
    bool updateParameters();

    /**
     * @brief Reset all modules
     */
    void reset();

    /**
     * @brief Emergency stop - immediately halt all traction control
     */
    void emergencyStop();

    /**
     * @brief Get combined performance metrics from all modules
     * @return Combined metrics structure
     */
    TractionModuleMetrics getCombinedMetrics() const;

    /**
     * @brief Set command fusion strategy
     * @param strategy Command fusion strategy
     */
    void setCommandFusionStrategy(std::function<bool(const std::vector<TractionCommand>&, TractionCommand&)> strategy);

    /**
     * @brief Enable/disable module arbitration
     * @param enabled Arbitration enabled state
     */
    void setArbitrationEnabled(bool enabled) { _arbitration_enabled = enabled; }

    /**
     * @brief Set maximum processing time per cycle
     * @param max_time_us Maximum time in microseconds
     */
    void setMaxProcessingTime(uint64_t max_time_us) { _max_processing_time_us = max_time_us; }

    /**
     * @brief Check if manager is initialized
     * @return true if initialized
     */
    bool isInitialized() const { return _initialized; }

    /**
     * @brief Get number of active modules
     * @return Number of enabled modules
     */
    size_t getActiveModuleCount() const;

private:
    // Module management
    std::vector<TractionControlModulePtr> _modules;
    std::map<std::string, TractionControlModulePtr> _module_map;
    bool _initialized{false};

    // Data providers and executors
    std::vector<std::shared_ptr<ITractionDataProvider>> _data_providers;
    std::vector<std::shared_ptr<ITractionCommandExecutor>> _command_executors;

    // Observers
    std::vector<std::shared_ptr<ITractionStateObserver>> _state_observers;
    std::vector<std::shared_ptr<ITractionCommandObserver>> _command_observers;

    // Processing control
    bool _arbitration_enabled{true};
    uint64_t _max_processing_time_us{10000}; // 10ms default
    bool _emergency_stop_active{false};

    // Command fusion
    std::function<bool(const std::vector<TractionCommand>&, TractionCommand&)> _command_fusion_strategy;

    // Performance tracking
    struct ManagerMetrics {
        uint64_t process_count{0};
        uint64_t success_count{0};
        uint64_t timeout_count{0};
        float avg_processing_time_us{0.0f};
        float max_processing_time_us{0.0f};
        uint64_t last_process_time{0};
    } _manager_metrics;

    /**
     * @brief Update traction state from data providers
     * @param state Output state structure
     * @return true if state updated successfully
     */
    bool updateTractionState(TractionState& state);

    /**
     * @brief Process all modules and collect their commands
     * @param state Current traction state
     * @param commands Output vector of commands from each module
     * @return true if processing successful
     */
    bool processModules(const TractionState& state, std::vector<TractionCommand>& commands);

    /**
     * @brief Arbitrate between multiple commands
     * @param commands Input commands from modules
     * @param final_command Output arbitrated command
     * @return true if arbitration successful
     */
    bool arbitrateCommands(const std::vector<TractionCommand>& commands, TractionCommand& final_command);

    /**
     * @brief Default command fusion strategy
     * @param commands Input commands from modules
     * @param final_command Output fused command
     * @return true if fusion successful
     */
    bool defaultCommandFusion(const std::vector<TractionCommand>& commands, TractionCommand& final_command);

    /**
     * @brief Priority-based command fusion strategy
     * @param commands Input commands from modules
     * @param final_command Output fused command
     * @return true if fusion successful
     */
    bool priorityBasedFusion(const std::vector<TractionCommand>& commands, TractionCommand& final_command);

    /**
     * @brief Weighted average command fusion strategy
     * @param commands Input commands from modules
     * @param final_command Output fused command
     * @return true if fusion successful
     */
    bool weightedAverageFusion(const std::vector<TractionCommand>& commands, TractionCommand& final_command);

    /**
     * @brief Execute final command through all executors
     * @param command Command to execute
     * @return true if execution successful
     */
    bool executeCommand(const TractionCommand& command);

    /**
     * @brief Notify state observers
     * @param state Current traction state
     */
    void notifyStateObservers(const TractionState& state);

    /**
     * @brief Notify command observers
     * @param command Current traction command
     */
    void notifyCommandObservers(const TractionCommand& command);

    /**
     * @brief Validate command before execution
     * @param command Command to validate
     * @return true if command is valid
     */
    bool validateCommand(const TractionCommand& command);

    /**
     * @brief Update manager performance metrics
     * @param processing_time_us Processing time in microseconds
     * @param success Processing success flag
     */
    void updateManagerMetrics(uint64_t processing_time_us, bool success);

    /**
     * @brief Sort modules by priority
     */
    void sortModulesByPriority();
};

/**
 * @brief Factory class for creating traction control modules
 */
class TractionControlModuleFactory {
public:
    /**
     * @brief Register a module creator function
     * @param module_type Module type name
     * @param creator Function that creates the module
     */
    static void registerModuleCreator(const std::string& module_type,
                                    std::function<TractionControlModulePtr()> creator);

    /**
     * @brief Create a module by type
     * @param module_type Module type name
     * @return Created module or nullptr if type not found
     */
    static TractionControlModulePtr createModule(const std::string& module_type);

    /**
     * @brief Get list of available module types
     * @return Vector of available module type names
     */
    static std::vector<std::string> getAvailableModuleTypes();

private:
    static std::map<std::string, std::function<TractionControlModulePtr()>> _creators;
};

/**
 * @brief Convenience macro for registering module creators
 */
#define REGISTER_TRACTION_MODULE(type_name, class_name) \
    namespace { \
        static bool _registered_##class_name = []() { \
            TractionControlModuleFactory::registerModuleCreator( \
                type_name, \
                []() -> TractionControlModulePtr { \
                    return std::make_shared<class_name>(); \
                } \
            ); \
            return true; \
        }(); \
    }

} // namespace traction_control
