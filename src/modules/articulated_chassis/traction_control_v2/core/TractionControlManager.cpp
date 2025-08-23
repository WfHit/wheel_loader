#include "TractionControlManager.hpp"
#include "../utils/TractionUtils.hpp"
#include <px4_platform_common/log.h>
#include <algorithm>

namespace traction_control {

// Static member initialization
std::map<std::string, std::function<TractionControlModulePtr()>> TractionControlModuleFactory::_creators;

TractionControlManager::TractionControlManager()
{
    // Set default command fusion strategy
    _command_fusion_strategy = std::bind(&TractionControlManager::defaultCommandFusion,
                                       this, std::placeholders::_1, std::placeholders::_2);
}

TractionControlManager::~TractionControlManager()
{
    // Emergency stop all modules
    emergencyStop();
}

bool TractionControlManager::initialize()
{
    if (_initialized) {
        return true;
    }

    PX4_INFO("Initializing traction control manager with %zu modules", _modules.size());

    // Initialize all registered modules
    for (auto& module : _modules) {
        if (!module->initialize()) {
            PX4_ERR("Failed to initialize module: %s", module->getName());
            return false;
        }
        PX4_INFO("Initialized module: %s v%s", module->getName(), module->getVersion());
    }

    // Sort modules by priority
    sortModulesByPriority();

    // Initialize manager metrics
    _manager_metrics = {};

    _initialized = true;
    PX4_INFO("Traction control manager initialized successfully");
    return true;
}

bool TractionControlManager::process(const TractionState& state, TractionCommand& command)
{
    if (!_initialized) {
        PX4_ERR("Manager not initialized");
        return false;
    }

    if (_emergency_stop_active) {
        // In emergency stop, just apply safe defaults
        command.reset();
        command.control.emergency_override = true;
        return executeCommand(command);
    }

    uint64_t start_time = hrt_absolute_time();

    // Update traction state from data providers
    TractionState updated_state = state;
    if (!updateTractionState(updated_state)) {
        PX4_WARN("Failed to update traction state from providers");
        // Continue with provided state
    }

    // Notify state observers
    notifyStateObservers(updated_state);

    // Process all modules
    std::vector<TractionCommand> module_commands;
    if (!processModules(updated_state, module_commands)) {
        _manager_metrics.timeout_count++;
        return false;
    }

    // Arbitrate/fuse commands
    if (_arbitration_enabled) {
        if (!arbitrateCommands(module_commands, command)) {
            PX4_WARN("Command arbitration failed");
            return false;
        }
    } else {
        // Use fusion strategy
        if (!_command_fusion_strategy(module_commands, command)) {
            PX4_WARN("Command fusion failed");
            return false;
        }
    }

    // Validate final command
    if (!validateCommand(command)) {
        PX4_WARN("Final command validation failed");
        return false;
    }

    // Execute command
    bool success = executeCommand(command);

    // Notify command observers
    notifyCommandObservers(command);

    // Update performance metrics
    uint64_t processing_time = hrt_absolute_time() - start_time;
    updateManagerMetrics(processing_time, success);

    return success;
}

bool TractionControlManager::registerModule(TractionControlModulePtr module)
{
    if (!module) {
        PX4_ERR("Cannot register null module");
        return false;
    }

    const char* module_name = module->getName();

    // Check if module already registered
    if (_module_map.find(module_name) != _module_map.end()) {
        PX4_WARN("Module %s already registered", module_name);
        return false;
    }

    _modules.push_back(module);
    _module_map[module_name] = module;

    PX4_INFO("Registered module: %s (priority: %d)", module_name, module->getPriority());

    // Re-sort by priority if already initialized
    if (_initialized) {
        sortModulesByPriority();
    }

    return true;
}

bool TractionControlManager::unregisterModule(const char* module_name)
{
    auto it = _module_map.find(module_name);
    if (it == _module_map.end()) {
        PX4_WARN("Module %s not found for unregistration", module_name);
        return false;
    }

    // Remove from vector
    auto module_ptr = it->second;
    _modules.erase(std::remove(_modules.begin(), _modules.end(), module_ptr), _modules.end());

    // Remove from map
    _module_map.erase(it);

    PX4_INFO("Unregistered module: %s", module_name);
    return true;
}

bool TractionControlManager::setModuleEnabled(const char* module_name, bool enabled)
{
    auto module = getModule(module_name);
    if (!module) {
        PX4_WARN("Module %s not found", module_name);
        return false;
    }

    module->setEnabled(enabled);
    PX4_INFO("Module %s %s", module_name, enabled ? "enabled" : "disabled");
    return true;
}

TractionControlModulePtr TractionControlManager::getModule(const char* module_name) const
{
    auto it = _module_map.find(module_name);
    return (it != _module_map.end()) ? it->second : nullptr;
}

std::vector<TractionControlModulePtr> TractionControlManager::getAllModules() const
{
    return _modules;
}

void TractionControlManager::registerDataProvider(std::shared_ptr<ITractionDataProvider> provider)
{
    if (provider) {
        _data_providers.push_back(provider);
        PX4_INFO("Registered data provider: %s", provider->getProviderName());
    }
}

void TractionControlManager::registerCommandExecutor(std::shared_ptr<ITractionCommandExecutor> executor)
{
    if (executor) {
        _command_executors.push_back(executor);
        PX4_INFO("Registered command executor: %s", executor->getExecutorName());
    }
}

void TractionControlManager::registerStateObserver(std::shared_ptr<ITractionStateObserver> observer)
{
    if (observer) {
        _state_observers.push_back(observer);
    }
}

void TractionControlManager::registerCommandObserver(std::shared_ptr<ITractionCommandObserver> observer)
{
    if (observer) {
        _command_observers.push_back(observer);
    }
}

bool TractionControlManager::updateParameters()
{
    bool all_success = true;

    for (auto& module : _modules) {
        if (!module->updateParameters()) {
            PX4_WARN("Failed to update parameters for module: %s", module->getName());
            all_success = false;
        }
    }

    return all_success;
}

void TractionControlManager::reset()
{
    PX4_INFO("Resetting all traction control modules");

    for (auto& module : _modules) {
        module->reset();
    }

    _manager_metrics = {};
    _emergency_stop_active = false;
}

void TractionControlManager::emergencyStop()
{
    PX4_WARN("Emergency stop activated");

    _emergency_stop_active = true;

    // Stop all command executors
    for (auto& executor : _command_executors) {
        if (executor && executor->isReady()) {
            executor->emergencyStop();
        }
    }
}

TractionModuleMetrics TractionControlManager::getCombinedMetrics() const
{
    TractionModuleMetrics combined = {};
    combined.timestamp = hrt_absolute_time();

    if (_modules.empty()) {
        return combined;
    }

    // Combine metrics from all modules
    for (const auto& module : _modules) {
        TractionModuleMetrics module_metrics = module->getMetrics();

        combined.process_count += module_metrics.process_count;
        combined.success_count += module_metrics.success_count;
        combined.error_count += module_metrics.error_count;
        combined.avg_processing_time_us += module_metrics.avg_processing_time_us;
        combined.max_processing_time_us = std::max(combined.max_processing_time_us,
                                                  module_metrics.max_processing_time_us);
        combined.cpu_usage_percent += module_metrics.cpu_usage_percent;
        combined.memory_usage_bytes += module_metrics.memory_usage_bytes;
        combined.effectiveness_score += module_metrics.effectiveness_score;
    }

    // Calculate averages
    size_t module_count = _modules.size();
    combined.avg_processing_time_us /= module_count;
    combined.cpu_usage_percent /= module_count;
    combined.effectiveness_score /= module_count;

    return combined;
}

void TractionControlManager::setCommandFusionStrategy(
    std::function<bool(const std::vector<TractionCommand>&, TractionCommand&)> strategy)
{
    _command_fusion_strategy = strategy;
}

size_t TractionControlManager::getActiveModuleCount() const
{
    size_t count = 0;
    for (const auto& module : _modules) {
        if (module->isEnabled()) {
            count++;
        }
    }
    return count;
}

bool TractionControlManager::updateTractionState(TractionState& state)
{
    bool updated = false;

    for (auto& provider : _data_providers) {
        if (provider && provider->isAvailable()) {
            TractionState provider_state;
            if (provider->getState(provider_state)) {
                // Merge provider state into main state
                // This is simplified - actual implementation would be more sophisticated
                if (provider_state.dynamics.dynamics_valid) {
                    state.dynamics = provider_state.dynamics;
                    updated = true;
                }
                if (provider_state.surface.conditions_valid) {
                    state.surface = provider_state.surface;
                    updated = true;
                }
                // ... merge other components as needed
            }
        }
    }

    return updated;
}

bool TractionControlManager::processModules(const TractionState& state, std::vector<TractionCommand>& commands)
{
    commands.clear();
    commands.reserve(_modules.size());

    uint64_t processing_start = hrt_absolute_time();

    for (auto& module : _modules) {
        if (!module->isEnabled()) {
            continue;
        }

        // Check processing time limit
        uint64_t elapsed = hrt_absolute_time() - processing_start;
        if (elapsed > _max_processing_time_us) {
            PX4_WARN("Processing timeout reached, skipping remaining modules");
            break;
        }

        TractionCommand module_command;
        if (module->process(state, module_command)) {
            commands.push_back(module_command);
        } else {
            PX4_DEBUG("Module %s processing failed", module->getName());
        }
    }

    return !commands.empty();
}

bool TractionControlManager::arbitrateCommands(const std::vector<TractionCommand>& commands,
                                              TractionCommand& final_command)
{
    if (commands.empty()) {
        return false;
    }

    // Simple arbitration: use highest priority (emergency override) command
    // or fall back to priority-based fusion

    // Check for emergency commands first
    for (const auto& cmd : commands) {
        if (cmd.control.emergency_override) {
            final_command = cmd;
            return true;
        }
    }

    // Use priority-based fusion for normal operation
    return priorityBasedFusion(commands, final_command);
}

bool TractionControlManager::defaultCommandFusion(const std::vector<TractionCommand>& commands,
                                                 TractionCommand& final_command)
{
    return weightedAverageFusion(commands, final_command);
}

bool TractionControlManager::priorityBasedFusion(const std::vector<TractionCommand>& commands,
                                                TractionCommand& final_command)
{
    if (commands.empty()) {
        return false;
    }

    // Find command with highest urgency
    const TractionCommand* best_command = nullptr;
    float highest_urgency = -1.0f;

    for (const auto& cmd : commands) {
        float urgency = cmd.getUrgency();
        if (urgency > highest_urgency) {
            highest_urgency = urgency;
            best_command = &cmd;
        }
    }

    if (best_command) {
        final_command = *best_command;
        return true;
    }

    return false;
}

bool TractionControlManager::weightedAverageFusion(const std::vector<TractionCommand>& commands,
                                                  TractionCommand& final_command)
{
    if (commands.empty()) {
        return false;
    }

    // Use CommandFusion utility for weighted average
    CommandFusion fusion;
    std::vector<float> weights(commands.size(), 1.0f / commands.size());

    return fusion.weightedAverageFusion(commands.data(), commands.size(),
                                      weights.data(), final_command);
}

bool TractionControlManager::executeCommand(const TractionCommand& command)
{
    bool all_success = true;

    for (auto& executor : _command_executors) {
        if (executor && executor->isReady()) {
            if (!executor->executeCommand(command)) {
                PX4_WARN("Command execution failed for executor: %s", executor->getExecutorName());
                all_success = false;
            }
        }
    }

    return all_success;
}

void TractionControlManager::notifyStateObservers(const TractionState& state)
{
    for (auto& observer : _state_observers) {
        if (observer) {
            try {
                observer->onStateUpdate(state);

                // Check for emergency conditions
                if (state.system.emergency_stop || !state.isCoreDataValid()) {
                    observer->onEmergencyCondition(state);
                }
            } catch (...) {
                PX4_WARN("State observer threw exception");
            }
        }
    }
}

void TractionControlManager::notifyCommandObservers(const TractionCommand& command)
{
    for (auto& observer : _command_observers) {
        if (observer) {
            try {
                observer->onCommandUpdate(command);

                if (!command.isValid()) {
                    observer->onCommandError(command, "Command validation failed");
                }
            } catch (...) {
                PX4_WARN("Command observer threw exception");
            }
        }
    }
}

bool TractionControlManager::validateCommand(const TractionCommand& command)
{
    SafetyValidator validator;
    return validator.validateCommand(command);
}

void TractionControlManager::updateManagerMetrics(uint64_t processing_time_us, bool success)
{
    _manager_metrics.process_count++;

    if (success) {
        _manager_metrics.success_count++;
    }

    // Update average processing time
    _manager_metrics.avg_processing_time_us =
        (_manager_metrics.avg_processing_time_us * (_manager_metrics.process_count - 1) +
         processing_time_us) / _manager_metrics.process_count;

    // Update max processing time
    if (processing_time_us > _manager_metrics.max_processing_time_us) {
        _manager_metrics.max_processing_time_us = processing_time_us;
    }

    _manager_metrics.last_process_time = hrt_absolute_time();
}

void TractionControlManager::sortModulesByPriority()
{
    std::sort(_modules.begin(), _modules.end(),
              [](const TractionControlModulePtr& a, const TractionControlModulePtr& b) {
                  return a->getPriority() > b->getPriority(); // Higher priority first
              });
}

// TractionControlModuleFactory implementation

void TractionControlModuleFactory::registerModuleCreator(
    const std::string& module_type,
    std::function<TractionControlModulePtr()> creator)
{
    _creators[module_type] = creator;
    PX4_INFO("Registered module creator for type: %s", module_type.c_str());
}

TractionControlModulePtr TractionControlModuleFactory::createModule(const std::string& module_type)
{
    auto it = _creators.find(module_type);
    if (it != _creators.end()) {
        return it->second();
    }

    PX4_WARN("Unknown module type: %s", module_type.c_str());
    return nullptr;
}

std::vector<std::string> TractionControlModuleFactory::getAvailableModuleTypes()
{
    std::vector<std::string> types;
    for (const auto& pair : _creators) {
        types.push_back(pair.first);
    }
    return types;
}

} // namespace traction_control
