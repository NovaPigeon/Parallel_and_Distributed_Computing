#ifndef UTILS_H
#define UTILS_H

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

int ungzip(std::string in_filename, std::string out_filename);

long getMemUsage();

struct Bbox {
  int x1, y1, x2, y2;
  Bbox(int x1, int y1, int x2, int y2) : x1(x1), y1(y1), x2(x2), y2(y2){};

  int area() const { return (x2 - x1) * (y2 - y1); }
};

class BboxOverlap {
public:
  void add_bbox(Bbox bbox) { bboxes.push_back(bbox); }

  double get_overlap_factor();

private:
  std::vector<Bbox> bboxes;
  std::vector<std::vector<Bbox>> bboxes_2d;

  class BboxComparator {
  public:
    bool operator()(const Bbox &lhs, const Bbox &rhs) const {
      if (lhs.x1 == rhs.x1) {
        return lhs.y1 < rhs.y1;
      }
      return lhs.x1 < rhs.x1;
    }
  };
};

#endif // UTILS_H