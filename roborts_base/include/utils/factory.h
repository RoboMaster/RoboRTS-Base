/****************************************************************************
 *  Copyright (C) 2021 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#ifndef ROBORTS_BASE_UTILS_FACTORY_H
#define ROBORTS_BASE_UTILS_FACTORY_H
#include <memory>
#include <stdio.h>
#include <iostream>
#include <functional>
#include <unordered_map>
#include "bind_this.h"

/**
 * @brief In order to conveniently using the sepecific module,
 *        such as gimbal, chassis, etc, it is nessessary to register different module
 *        classes to the module container in ModuleFactory class. The example is at
 *        the end of this program.
 * @tparam ModuleBase The base class of algorithm.
 * @tparam Args Module pack
 */
template<class ModuleBase, typename... Args>
class ModuleFactory
{
 public:
  using ModuleHash = std::unordered_map<std::string, std::function<std::unique_ptr<ModuleBase>(Args...)>>;

  /**
   * @brief The only way to get ModuleHash.
   * @return Ref of ModuleHash
   */
  static ModuleHash& GetModuleHash(){
    static ModuleHash module_hash;
    return module_hash;
  }

  static std::vector<std::string> GetModuleName(){
    ModuleHash& module_hash = GetModuleHash();
    std::vector<std::string> keys;
    keys.reserve(module_hash.size());
    for(auto kv : module_hash) {
      keys.push_back(kv.first);
    }
    return keys;
  }

  /**
   * @brief Register specific module to the module container called ModuleHash.
   * @tparam ParamType Type emplate of parameters
   * @param module_name Module name, which must be the same with the variable defined in ".prototxt" file.
   * @param args Parameter pack
   * @return Return true if register successfully, false otherwith.
   */
  template <typename ParamType>
  static bool Register(const std::string module_name,
                       ParamType&& args){
    ModuleHash& module_hash = GetModuleHash();
    auto factory_iter = module_hash.find(module_name);
    if(factory_iter == module_hash.end()) {
      module_hash.emplace(std::make_pair(module_name, std::forward<ParamType>(args)));
//      std::cout << module_name << " registered successfully!" << std::endl;
    }
    else
      std::cerr << module_name << " has been registered!" << std::endl;
    return true;
  }

  /**
   * @brief Unregister module from the module container.
   * @param module_name Module name, which must be the same with the variable defined in ".prototxt" file.
   * @return Return true if register successfully, false otherwith.
   */
  static bool UnRegister(const std::string module_name){
    ModuleHash& module_hash = GetModuleHash();
    auto factory_iter = module_hash.find(module_name);
    if(factory_iter != module_hash.end()){
      module_hash.erase(module_name);
      return true;
    }
    else{
      std::cout << "Failed to unregister module, it is a unregistered alrorithm."
                << module_name << std::endl;
      return false;
    }
  }

  /**
   * @brief Create an module class that has been registered before.
   * @param module_name Module name, which must be the same with the variable defined in ".prototxt" file.
   * @param args Parameter pack
   * @return The base class unique_ptr that point to Module corresponding to the module name.
   */
  static std::unique_ptr<ModuleBase> CreateModule(const std::string module_name,
                                                        Args... args) {
    ModuleHash& module_hash = GetModuleHash();

    auto factory_iter = module_hash.find(module_name);
    if(factory_iter == module_hash.end()){
//      std::cout << "Can't creat module " << module_name <<
//                ", because you haven't register it!" << std::endl;
      return nullptr;
    }
    else{
      return (factory_iter->second)(std::forward<Args>(args)...);
    }
  }

 private:
  ModuleFactory(){}
};

template<typename ModuleBase, typename Module, typename... Args>
class ModuleRegister{
 public:
  /**
   * @brief Constructor function of ModuleRegister class.
   * @param module_name Module name, which must be the same with the variable defined in ".prototxt" file.
   */
  explicit ModuleRegister(std::string module_name) {
    Register(module_name, make_int_sequence<sizeof...(Args)>{});
  }
  /**
   * @brief Create an unique_ptr pointer of Module corresponding to the module name.
   * @param data Parameter pack
   * @return The base class unique_ptr that point to Module corresponding to the module name.
   */
  static std::unique_ptr<Module> create(Args&&... data)
  {
    return std::make_unique<Module>(std::forward<Args>(data)...);
  }

 private:
  /**
   * @brief Using std::bind to create a std::function, than pass it to Register function befined in ModulenFactory
   *        class
   * @tparam Is Parameter pack
   * @param module_name Module name, which must be the same with the variable defined in ".prototxt" file.
   */
  template <int... Is>
  void Register(std::string module_name, int_sequence<Is...>) {
    auto function = std::bind(&ModuleRegister<ModuleBase, Module, Args...>::create,
                              std::placeholder_template<Is>{}...);
    ModuleFactory<ModuleBase, Args...>::Register(module_name, function);
  }
};
/**
 * @brief It is convenient to register specific module using this Macro.
 * @example
 * class BaseClass {
 *   public:
 *    BaseClass(int num){
 *      num_ = num;
 *    }
 *    virtual void Excute() = 0;
 *   public:
 *    int num_;
 *  };
 *
 *  class A: public BaseClass {
 *   public:
 *    A(int num): BaseClass(num) {}
 *    void Excute(){
 *      std::cout << "A, you are so beautiful!" << std::endl;
 *    }
 *  };
 *  REGISTER_MODULE(BaseClass, "a", A, int);
 *
 *  class B: public BaseClass {
 *   public:
 *    B(int num): BaseClass(num){}
 *    void Excute() {
 *      std::cout << "B, you are so beautiful!" << std::endl;
 *    }
 *  };
 *  REGISTER_MODULE(BaseClass, "b", B, int);
 *
 *  int main() {
 *    auto base_class = ModuleFactory<BaseClass, int>::CreateModule("a", 3);
 *    base_class->Excute();
 *  }
 */
#define REGISTER_MODULE_NAME(name) register_##name##_module
#define REGISTER_MODULE(ModuleBase, ModuleName, Module, ...)                          \
        ModuleRegister<ModuleBase, Module, ##__VA_ARGS__> REGISTER_MODULE_NAME(Module)(ModuleName)

#endif // ROBORTS_BASE_UTILS_FACTORY_H