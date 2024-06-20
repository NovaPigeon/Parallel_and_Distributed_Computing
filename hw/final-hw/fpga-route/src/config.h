#ifndef CONFIG_H
#define CONFIG_H

#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>

class Config {
public:
  Config() {}
  void parse(const std::string &filename);

  /** If use parallel bi-partition */
  bool use_bipartition_par = false;
  /** If use bidirectional search */
  bool use_bidirectional = false;
  // If use bidirectional search, then the following parameters are used
  bool bidirectional_par = false;
  bool bidirectional_adapt = false;
  unsigned long long ba_switch_nodes_popped =
      std::numeric_limits<unsigned long long>::max();
  double ba_switch_route_time = std::numeric_limits<double>::max();

  bool bidirectional_adapt_old = false;
  long use_ba_upper_thresh = 20000;
  long use_ba_lower_thresh = 0;

  std::string connections_log;

  /** Initial present congestion penalty factor */
  double initial_present_congestion_factor = 0.5;
  /** The multiplier factor for present congestion update */
  double present_congestion_multiplier = 2;
  /** Wirelength-driven weighting factor */
  double wirelength_weight = 0.8;
  /** Allowed max number of routing iterations */
  int max_iterations = 1000;
  /** Historical congestion penalty factor */
  double historical_congestion_factor = 1;
  /** Routing bounding box constraint */
  bool use_bounding_box = true;
  /** Initial bounding box extension range to the left and right */
  int bounding_box_extension_x = 3;
  /** Initial bounding box extension range to the top and bottom */
  int bounding_box_extension_y = 15;
  /** Further enlarge the bounding box along with routing iterations by the
   * extension X and Y increments */
  bool enlarge_bounding_box = false;
  /** Incremental extension of the bounding box in the horizontal direction to
   * the left and right */
  int extension_x_increment = 1;
  /** Incremental extension of the bounding box in the vertical direction to the
   * top and bottom */
  int extension_y_increment = 2;

  std::unordered_map<std::string, std::function<void(std::string)>> setters = {
      {"use_bipartition_par",
       [this](std::string value) {
         use_bipartition_par =
             (value == "true" || value == "True" || value == "1");
       }},
      {"use_bidirectional",
       [this](std::string value) {
         use_bidirectional =
             (value == "true" || value == "True" || value == "1");
       }},
      {"bidirectional_par",
       [this](std::string value) {
         bidirectional_par =
             (value == "true" || value == "True" || value == "1");
       }},
      {"bidirectional_adapt",
       [this](std::string value) {
         bidirectional_adapt =
             (value == "true" || value == "True" || value == "1");
       }},
      {"bidirectional_adapt_old",
       [this](std::string value) {
         bidirectional_adapt_old =
             (value == "true" || value == "True" || value == "1");
       }},
      {"use_ba_upper_thresh",
       [this](std::string value) { use_ba_upper_thresh = std::stol(value); }},
      {"use_ba_lower_thresh",
       [this](std::string value) { use_ba_lower_thresh = std::stol(value); }},
      {"ba_switch_nodes_popped",
       [this](std::string value) {
         ba_switch_nodes_popped = std::stoull(value);
       }},
      {"ba_switch_route_time",
       [this](std::string value) { ba_switch_route_time = std::stod(value); }},
      {"connections_log",
       [this](std::string value) { connections_log = value; }},
      {"initial_present_congestion_factor",
       [this](std::string value) {
         initial_present_congestion_factor = std::stod(value);
       }},
      {"present_congestion_multiplier",
       [this](std::string value) {
         present_congestion_multiplier = std::stod(value);
       }},
      {"wirelength_weight",
       [this](std::string value) { wirelength_weight = std::stod(value); }},
      {"max_iterations",
       [this](std::string value) { max_iterations = std::stoi(value); }},
      {"historical_congestion_factor",
       [this](std::string value) {
         historical_congestion_factor = std::stod(value);
       }},
      {"use_bounding_box",
       [this](std::string value) {
         use_bounding_box =
             (value == "true" || value == "True" || value == "1");
       }},
      {"bounding_box_extension_x",
       [this](std::string value) {
         bounding_box_extension_x = std::stoi(value);
       }},
      {"bounding_box_extension_y",
       [this](std::string value) {
         bounding_box_extension_y = std::stoi(value);
       }},
      {"enlarge_bounding_box",
       [this](std::string value) {
         enlarge_bounding_box =
             (value == "true" || value == "True" || value == "1");
       }},
      {"extension_x_increment",
       [this](std::string value) { extension_x_increment = std::stoi(value); }},
      {"extension_y_increment",
       [this](std::string value) { extension_y_increment = std::stoi(value); }},
  };
};

// #define USE_SHARED_PTR
#define OMP_MAX_THREADS 4

enum PBA_MODE { NONE, FORWARD, BACKWARD };

#ifdef USE_SHARED_PTR
// >>> BEGIN USE_SHARED_PTR
#define DECLARE_PTR(T)                                                         \
  class T;                                                                     \
  typedef std::shared_ptr<T> T##Ptr;

template <typename T, typename... Args>
inline std::shared_ptr<T> new_ptr(Args... args) {
  return std::make_shared<T>(args...);
}

template <typename T> inline void reset_ptr(std::shared_ptr<T> &ptr) {
  ptr.reset();
  ptr = nullptr;
}
template <typename T> inline void delete_ptr(std::shared_ptr<T> &ptr) {
  ptr.reset();
  ptr = nullptr;
}
// <<< END USE_SHARED_PTR
#else
// >>> BEGIN DON'T USE_SHARED_PTR
#define DECLARE_PTR(T)                                                         \
  class T;                                                                     \
  typedef T *T##Ptr;

template <typename T, typename... Args> inline T *new_ptr(Args... args) {
  return new T(args...);
}

template <typename T> inline void reset_ptr(T *&ptr) { ptr = nullptr; }
template <typename T> inline void delete_ptr(T *&ptr) {
  delete ptr;
  ptr = nullptr;
}
// <<< END DON'T USE_SHARED_PTR
#endif

template <typename T> inline void clear_container(T &q) {
  T empty;
  std::swap(q, empty);
}

#define PTR_FACTORY(T)                                                         \
  template <typename... Args> static T##Ptr create(Args... args) {             \
    return new_ptr<T>(args...);                                                \
  }
#define PTR_RECYCLE(T)                                                         \
  template <typename... Args> static void destroy(T##Ptr &entity) {            \
    delete_ptr(entity);                                                        \
  }

#define ACCESSOR_RO(Type, member)                                              \
private:                                                                       \
  Type member##_;                                                              \
                                                                               \
public:                                                                        \
  const Type &member() const { return member##_; }

#define ACCESSOR_RW(Type, member)                                              \
private:                                                                       \
  Type member##_;                                                              \
                                                                               \
public:                                                                        \
  const Type &get_##member() const { return member##_; }                       \
  void set_##member(const Type &value) { member##_ = value; }

#define ACCESSOR_RW_BOOL(member)                                               \
private:                                                                       \
  bool is_##member##_;                                                         \
                                                                               \
public:                                                                        \
  bool is_##member() const { return is_##member##_; }                          \
  void set_##member(bool value = true) { is_##member##_ = value; }

#define ACCESSOR_RW_BOOL_DEFAULT(member, init_value)                           \
private:                                                                       \
  bool is_##member##_ = init_value;                                            \
                                                                               \
public:                                                                        \
  bool is_##member() const { return is_##member##_; }                          \
  void set_##member(bool value = true) { is_##member##_ = value; }

#endif // CONFIG_H