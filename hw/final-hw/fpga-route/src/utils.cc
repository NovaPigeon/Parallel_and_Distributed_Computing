#include "utils.h"
#include <filesystem>
#include <fstream>
#include <ios>
#include <iostream>
#include <unistd.h>
#include <zlib.h>

#include <log4cplus/logger.h>
#include <log4cplus/loggingmacros.h>

using namespace std;

static log4cplus::Logger logger = log4cplus::Logger::getInstance("ace.utils");

int ungzip(string in_filename, string out_filename) {
  if (std::filesystem::exists(out_filename)) {
    LOG4CPLUS_WARN_FMT(logger, "Reuse previously unzipped %s",
                       out_filename.c_str());
    return 0;
  }

  LOG4CPLUS_INFO_FMT(logger, "Start ungzip %s to %s", in_filename.c_str(),
                     out_filename.c_str());
  gzFile gzfile = gzopen(in_filename.c_str(), "rb");
  if (!gzfile) {
    LOG4CPLUS_ERROR_FMT(logger, "Could not open %s", in_filename.c_str());
    return 1;
  }

  ofstream outfile(out_filename.c_str(), ofstream::binary);
  if (!outfile) {
    LOG4CPLUS_ERROR_FMT(logger, "Could not open %s", out_filename.c_str());
    gzclose(gzfile);
    return 1;
  }

  char buffer[4096];
  int bytes_read;
  while ((bytes_read = gzread(gzfile, buffer, sizeof(buffer))) > 0) {
    outfile.write(buffer, bytes_read);
  }

  gzclose(gzfile);
  outfile.close();
  LOG4CPLUS_INFO(logger, "Ungzip success");
  return 0;
}

long getMemUsage() {
  std::ifstream procFile("/proc/self/statm");
  long size;     //   total program size
  long resident; //   resident set size
  long share;    //   shared pages
  long text;     //   text (code)
  long lib;      //   library
  long data;     //   data/stack
  long dt;       //   dirty pages (unused in Linux 2.6)

  procFile >> size >> resident >> share >> text >> lib >> data >>
      dt; // don't care about the rest
  procFile.close();

  long pageSize =
      sysconf(_SC_PAGE_SIZE); // in case x86-64 is configured to use 2MB pages
  return resident * pageSize;
}

double BboxOverlap::get_overlap_factor() {
  if (bboxes.empty()) {
    return 0;
  }
  long long total_area = 0;
  long long overlap_area = 0;

  std::sort(bboxes.begin(), bboxes.end(), BboxComparator());

  int cur_x1 = -100;
  for (auto bbox : bboxes) {
    total_area += bbox.area();

    if (bbox.x1 != cur_x1) {
      cur_x1 = bbox.x1;
      bboxes_2d.push_back(vector<Bbox>());
    }
    bboxes_2d.back().push_back(bbox);
  }

  for (size_t bbox_i = 0; bbox_i < bboxes_2d.size(); bbox_i++) {
    for (size_t bbox_j = 0; bbox_j < bboxes_2d[bbox_i].size(); bbox_j++) {
      auto bbox = bboxes_2d[bbox_i][bbox_j];

      for (size_t bbox_j1 = bbox_j + 1; bbox_j1 < bboxes_2d[bbox_i].size();
           bbox_j1++) {
        auto _bbox = bboxes_2d[bbox_i][bbox_j1];
        if (_bbox.y1 >= bbox.y2) {
          break;
        }
        overlap_area += (min(bbox.x2, _bbox.x2) - max(bbox.x1, _bbox.x1)) *
                        (min(bbox.y2, _bbox.y2) - max(bbox.y1, _bbox.y1));
      }

      for (size_t bbox_i1 = bbox_i + 1; bbox_i1 < bboxes_2d.size(); bbox_i1++) {
        if (bboxes_2d[bbox_i1][0].x1 >= bbox.x2) {
          break;
        }

        for (size_t bbox_j1 = 0; bbox_j1 < bboxes_2d[bbox_i1].size();
             bbox_j1++) {
          auto _bbox = bboxes_2d[bbox_i1][bbox_j1];
          if (_bbox.y1 >= bbox.y2) {
            break;
          }
          if (_bbox.y2 <= bbox.y1) {
            continue;
          }
          overlap_area += (min(bbox.x2, _bbox.x2) - max(bbox.x1, _bbox.x1)) *
                          (min(bbox.y2, _bbox.y2) - max(bbox.y1, _bbox.y1));
        }
      }
    }
  }

  return (double)overlap_area / total_area;
}