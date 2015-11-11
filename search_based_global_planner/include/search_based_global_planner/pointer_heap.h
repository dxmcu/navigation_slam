/* Copyright(C) Gaussian Automation. All rights reserved.
*/

/**
 * @file pointer_heap.h
 * @brief 指针堆仅保存指针类型，且不负责指针分配和释放
 *        指针必须可以访问heap_index字段，如p->heap_index
 * @author cameron<chenkan@gs-robot.com>
 * @version 1.0.0.0
 * @date 2015-09-09
 */

#ifndef SEARCH_BASED_GLOBAL_PLANNER_INCLUDE_SEARCH_BASED_GLOBAL_PLANNER_POINTER_HEAP_H_
#define SEARCH_BASED_GLOBAL_PLANNER_INCLUDE_SEARCH_BASED_GLOBAL_PLANNER_POINTER_HEAP_H_

#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <vector>
#include <algorithm>
#include <memory>

#define __must_check

/**
 * @brief 用于在编译时验证Heap保存的结构体指针包含heap_index域
 **/
template <typename T>
struct HasHeapIndexConcept {
  T a;
  void __constraints() {
    // const char*(T::*x)() = &T::getVal;
    // a.getVal(); // 确定有成员函数 getVal
    T b;
    uint32_t tmp = a.heap_index;         ///< 可读
    a.heap_index = tmp;                  ///< 可写
  }
};

/**
 * @defgroup PointerHeap 返回值代码
 * @{
 */
#define PTRHEAP_OK                       (0)        ///< 操作成功
#define PTRHEAP_PARAM_NULL              (-1)        ///< 参数为NULL
#define PTRHEAP_PARAM_INV_INDEX         (-2)        ///< 参数INDEX越界
#define PTRHEAP_DATA_NOT_EXIST          (-3)        ///< 数据项不存在
#define PTRHEAP_MEM_EXHAUST             (-4)        ///< 内存不足
#define PTRHEAP_MEM_ERROR               (-5)        ///< 内存错误
#define PTRHEAP_EMPTY                   (-6)        ///< 空堆操作
#define PTRHEAP_FATAL                   (-7)        ///< 严重错误
#define PTRHEAP_UNKNOWN_ERROR           (-8)        ///< 未知错误
/**@}*/

/**
 * @brief 最小堆，仅保存结构体的指针，
 * 同时维护结构体的heap_index标识指针在堆中的位置，便于删除。
 * 使用“shrink-to-fit惯用法：container<T>(c).swap(c)”来释放多余的内存
 **/
template<typename _Tp, typename _Comp, typename _Alloc = std::allocator<_Tp> >
class PointerHeap {
  /**
   * concept requirements
   * @brief 验证_Tp为指针类型, 且_Tp->heap_index可读可写
   **/
  __glibcxx_class_requires(_Tp, HasHeapIndexConcept);
  /**
   * concept requirements
   * @brief 验证_Comp重载()运算符，接收2个_Tp参数返回bool值
   **/
  __glibcxx_class_requires4(_Comp, bool, _Tp, _Tp, _BinaryFunctionConcept);

 public:
  typedef     _Tp             _Type;      ///< 数据类型
  typedef     _Comp           _Compare;   ///< 比较函数
  typedef typename std::vector<_Type, _Alloc>::iterator iterator;  ///< iterator type

 private:
  static constexpr double          __FACTOR = 2;   ///< vector free memory factor
  _Compare                         __compare;      ///< 比较函数实例
  std::vector<_Type, _Alloc>       __container;    ///< 使用std::vector作为容器

 public:
  /**
   * @brief Constructor with _Compare() and std::vector<_Tp,_Alloc>()
   **/
  PointerHeap(): __compare(_Compare()), __container(std::vector<_Tp, _Alloc>()) {
    // do nothing
  }

  /**
   * @brief Constructor with _Compare() and std::vector<_Tp,_Alloc>(capacity)
   *
   * @param [in/out] capacity   : uint32_t
   **/
  explicit PointerHeap(uint32_t capacity)
    : __compare(_Compare()),
      __container(std::vector<_Tp, _Alloc>(capacity)) {
    __container.resize(0);
    // do nothing
  }

  /**
   * @brief Retrieve begin iterator of __container
   *
   * @return iterator
   * @retval Begin iterator of __container
   */
  iterator begin() {
    return __container.begin();
  }

  /**
   * @brief Retrieve end iterator of __container
   *
   * @return iterator
   * @retval End iterator of __container
   */
  iterator end() {
    return __container.end();
  }

  /**
   * @brief Manually maintain heap after changing heap element
   *
   * @return uint32_t
   * @retval  PTRHEAP_OK
   */
  int make_heap() {
    if (size() <= 1) {
      return PTRHEAP_OK;
    }
    for (int i = (size() - 2) / 2; i >= 0; --i) {
      __adjust_down(i);
    }
    return PTRHEAP_OK;
  }

  /**
   * @brief Manually adjust heap after changing heap element
   *
   * @return uint32_t
   * @retval  PTRHEAP_OK
   *          others  -  return from contain()
   */
  int adjust(_Type& data) {
    int retval = contain(data);
    if (retval != PTRHEAP_OK) return retval;

    if (data->heap_index == 0) {
      __adjust_down(0);
    } else {
      uint32_t index = data->heap_index;
      uint32_t parent = (index - 1) / 2;

      if (__compare(data, __container[parent])) {
        __adjust_up(index);
      } else {
        __adjust_down(index);
      }
    }
    return PTRHEAP_OK;
  }

  /**
   * @brief Remove all elements
   *
   * @return  uint32_t
   * @retval  PTRHEAP_OK
   **/
  int clear() {
    __container.clear();
    __shrink();
    return PTRHEAP_OK;
  }

  /**
   * @brief Return the size of this heap
   *
   * @return  int32_t
   **/
  inline size_t size() const {
    return __container.size();
  }

  /**
   * @brief Return if heap is empty
   *
   * @return  bool
   **/
  bool empty() const {
    return __container.size() == 0;
  }

  /**
   * @brief Return the top element of the heap
   *
   * @return  _Type
   * @retval  NULL - when empty
   **/
  _Type top() const {
    if (size() <= 0) {
      return NULL;
    } else {
      return __container[0];
    }
  }

  /**
   * @brief Pop the element at the top of the heap
   *
   * @return  int32_t
   * @retval  PTRHEAP_OK
   *          PTRHEAP_EMPTY  -   size <= 0
   **/
  int pop() {
    if (size() <= 0) {
      return PTRHEAP_EMPTY;
    }

    std::swap(__container[0], __container[__container.size() - 1]);
    __container[0]->heap_index = 0;
    __container[__container.size() - 1]->heap_index = __container.size() - 1;
    __container.pop_back();

    if (__container.size() > 0) {
      __adjust_down(0);
    }

    __shrink();
    return PTRHEAP_OK;
  }

  /**
   * @brief Pop the top element and copy it to data
   *
   * @param [in/out] data   : _Type&
   * @return  int32_t
   * @retval  PTRHEAP_OK
   *          PTRHEAP_EMPTY  -   size <= 0
   **/
  int pop(_Type& data) __must_check {  // NOLINT
    if (size() <= 0) {
      return PTRHEAP_EMPTY;
    }

    data = __container[0];

    std::swap(__container[0], __container[__container.size() - 1]);
    __container[0]->heap_index = 0;
    __container[__container.size() - 1]->heap_index = __container.size() - 1;

    __container.pop_back();

    if (__container.size() > 0) {
      __adjust_down(0);
    }

    __shrink();
    return PTRHEAP_OK;
  }

  /**
   * @brief check if :
   *      the element specified by data->heap_index equals data.
   *
   * @param [in/out] data   : const _Type&
   * @return  int32_t
   * @retval  PTRHEAP_OK
   *          PTRHEAP_PARAM_NULL
   *          PTRHEAP_PARAM_INV_INDEX
   *          PTRHEAP_DATA_NOT_EXIST
   **/
  int contain(const _Type& data) {
    if (data == NULL) {
      return PTRHEAP_PARAM_NULL;
    }

    if (size() == 0) {
      return PTRHEAP_EMPTY;
    }

    if (data->heap_index < 0 || (size_t)data->heap_index >= size()) {
      return PTRHEAP_PARAM_INV_INDEX;
    }

    if (data != __container[data->heap_index]) {
      return PTRHEAP_DATA_NOT_EXIST;
    } else {
      return PTRHEAP_OK;
    }
  }

  /**
   * @brief Push the new element to the heap
   *
   * @param [in/out] data   : const _Type&
   * @return  int32_t
   * @retval  PTRHEAP_OK
   *          PTRHEAP_PARAM_NULL
   *          PTRHEAP_MEM_EXHAUST
   **/
  int push(const _Type& data) __must_check {
    if (data == NULL) {
      return PTRHEAP_PARAM_NULL;
    }

    try {
      __container.push_back(data);
    } catch (std::bad_alloc& e) {
      // WARNING("std::bad_alloc[%s] no enough memory",e.what());
      return PTRHEAP_MEM_EXHAUST;
    } catch (...) {
      // WARNING("UNKNOWN exception was thrown! BUG!");
      return PTRHEAP_MEM_EXHAUST;
    }

    __container[__container.size() - 1]->heap_index = __container.size() - 1;
    __adjust_up(__container.size() - 1);
    return PTRHEAP_OK;
  }

  /**
   * @brief Erase the element specified by data->heap_index
   *
   * @param [in/out] data   : const _Type&
   * @return  int32_t
   * @retval  PTRHEAP_OK
   *          PTRHEAP_PARAM_NULL
   *          PTRHEAP_PARAM_INV_INDEX
   *          PTRHEAP_DATA_NOT_EXIST
   **/
  int erase(const _Type& data) __must_check {
    if (data == NULL) {
      return PTRHEAP_PARAM_NULL;
    }

    if (size() == 0) {
      return PTRHEAP_EMPTY;
    }

    if (data->heap_index < 0 || (size_t)data->heap_index >= size()) {
      return PTRHEAP_PARAM_INV_INDEX;
    }

    if (data != __container[data->heap_index]) {
      // assert(0);
      return PTRHEAP_DATA_NOT_EXIST;
    }

    return __erase(uint32_t(data->heap_index));
  }

  /**
   * @brief Return the reference of __container
   *
   * @return  vector<_Type>&
   **/
  std::vector<_Type>& vector() {
    return __container;
  }

  /**
   * @brief 打印错误代码对应的文字描述
   * @param nError 错误代码
   */
  void print_error(int eid) {
    printf("%s", __string_error(eid));
    printf("\n");
  }

 private:
  /**
   * @brief 将错误代码转换为对应的文字描述
   * @param eid错误代码
   * @return 返回与eid对应的描述字符串
   */
  const char* __string_error(int eid) {
    static const char* ERROR_MSG[] = {
      "Success",
      "Null pointer in parameter(s)",
      "Invalid heap_index",
      "data not match",
      "Memory not enough",
      "Memory error",
      "Empty heap: can't pop",
      "Fatal error",
      "Unknown error",
    };

    if (eid < PTRHEAP_UNKNOWN_ERROR || eid > 0) {
      return ERROR_MSG[-PTRHEAP_UNKNOWN_ERROR];
    }

    return ERROR_MSG[-eid];
  }

  /**
   * @brief 当size小于capacity的50%，将vector内存变小
   *
   * @return  void
   **/
  inline void __shrink() {
    if (__container.size() * __FACTOR < __container.capacity()) {
      uint32_t old_size = __container.size();
      uint32_t new_capacity = old_size * 3 / 2;
      uint32_t fill_size = new_capacity - old_size;

      try {
        __container.insert(__container.end(), fill_size, NULL);
        std::vector<_Type, _Alloc>(__container).swap(__container);
        __container.resize(old_size);
      } catch (std::bad_alloc& e) {
        // WARNING("std::bad_alloc[%s] no enough memory",e.what());
        __container.resize(old_size);
      } catch (...) {
        // WARNING("UNKNOWN exception was thrown! BUG!");
        __container.resize(old_size);
      }
    }
  }

  /**
   * @brief Adjust the element at __container[index] upward
   *
   * @param [in/out] index   : const uint32_t&     the index of the element
   * @return  void
   **/
  void __adjust_up(const uint32_t& index) {
    assert(index < size());
    uint32_t parent = (index - 1) / 2;
    uint32_t curpos = index;
    _Type value = __container[index];

    while (parent < curpos
           && __compare(value, __container[parent])) {
      __container[curpos] = __container[parent];
      __container[curpos]->heap_index = curpos;
      curpos = parent;
      parent = (curpos - 1) / 2;
    }

    __container[curpos] = value;
    __container[curpos]->heap_index = curpos;
  }

  /**
   * @brief Adjust the element at __container[index] downward
   *
   * @param [in/out] index   : const uint32_t&     the index of the element
   * @return  void
   **/
  void __adjust_down(const uint32_t& index) {
    assert(index < size());
    _Type value = __container[index];
    uint32_t curpos = index;
    uint32_t child = index * 2 + 1;

    while (child + 1 < __container.size()) {
      // compare between left child and right child
      if (__compare(__container[child + 1], __container[child])) {
        ++child;
      }

      // compare between smaller child and value
      if (__compare(value, __container[child])) {
        // find the right position
        __container[curpos] = value;
        __container[curpos]->heap_index = curpos;
        break;
      } else {
        __container[curpos] = __container[child];
        __container[curpos]->heap_index = curpos;
        curpos = child;
        child = curpos * 2 + 1;
      }
    }

    if (child + 1 == __container.size()) {
      if (__compare(__container[child], value)) {
        __container[curpos] = __container[child];
        __container[curpos]->heap_index = curpos;
        __container[child] = value;
        __container[child]->heap_index = child;
      } else {
        __container[curpos] = value;
        __container[curpos]->heap_index = curpos;
      }
    } else {
      __container[curpos] = value;
      __container[curpos]->heap_index = curpos;
    }
  }

  /**
   * @brief Erase the element specified by heap_index
   *
   * @param [in/out] index   : const int32_t&
   * @return  int32_t
   * @retval  PTRHEAP_OK
   **/
  int __erase(const uint32_t& index) {
    if (index == size() - 1) {
      __container.pop_back();
      return PTRHEAP_OK;
    }

    size_t sz = size();
    std::swap(__container[index], __container[sz - 1]);
    __container[index]->heap_index = index;
    __container[sz - 1]->heap_index = sz - 1;

    if (__compare(__container[index], __container[sz - 1])) {
      __container.pop_back();
      __adjust_up(index);
    } else {
      __container.pop_back();
      __adjust_down(index);
    }

    __shrink();
    return PTRHEAP_OK;
  }
};

#endif  // SEARCH_BASED_GLOBAL_PLANNER_INCLUDE_SEARCH_BASED_GLOBAL_PLANNER_POINTER_HEAP_H_
