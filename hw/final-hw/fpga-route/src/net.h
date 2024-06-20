#ifndef NET_H
#define NET_H

#include "config.h"
#include "device.h"
#include <vector>

DECLARE_PTR(Connection)
DECLARE_PTR(Net)

class Net {
public:
  PTR_FACTORY(Net)
  PTR_RECYCLE(Net)

  Net(int id_) : id_(id_), double_hpwl_((1 + 1) * 2) {}

  void add_connection(ConnectionPtr connection) {
    connections_.push_back(connection);
  }

  void compute();

  ACCESSOR_RO(int, id)
  ACCESSOR_RO(std::vector<ConnectionPtr>, connections)
  ACCESSOR_RO(double, x_center)
  ACCESSOR_RO(double, y_center)
  ACCESSOR_RO(int, double_hpwl)
};

#endif // NET_H