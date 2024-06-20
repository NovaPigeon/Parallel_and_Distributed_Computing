#ifndef DEVICE_H
#define DEVICE_H

#include "config.h"
#include "intentcode.h"
#include "mapped-file.h"
#include "utils.h"
#include <cassert>
#include <iostream>
#include <memory>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "DeviceResources.capnp.h"
#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include <capnp/serialize.h>

template <typename T, typename U, typename V>
inline bool
find_in_2d_map(const std::unordered_map<T, std::unordered_map<U, V>> &my_map,
               const T &key1, const U &key2) {
  return my_map.contains(key1) && my_map.at(key1).contains(key2);
}

struct PipData { // only used when writing results
  uint32_t tile_name_id;
  uint32_t wire0_name_id;
  uint32_t wire1_name_id;
  bool is_forward;
};

namespace device {

DECLARE_PTR(Device)
DECLARE_PTR(TileType)
DECLARE_PTR(SiteType)
DECLARE_PTR(SiteTypeInst)
DECLARE_PTR(Tile)
DECLARE_PTR(Node)
DECLARE_PTR(Wire)
DECLARE_PTR(Site)
DECLARE_PTR(SitePin)
DECLARE_PTR(SitePinInst)
DECLARE_PTR(PipInst)

typedef std::pair<TilePtr, WirePtr> TileWire;

struct PtrHasher {
  template <typename T> std::size_t operator()(const T &ptr) const {
    return std::hash<uint32_t>{}(ptr->name());
  }
};
struct PtrEqual {
  template <typename T> bool operator()(const T &lhs, const T &rhs) const {
    return lhs->name() == rhs->name();
  }
};
typedef std::unordered_map<WirePtr, IntentCode, PtrHasher, PtrEqual>
    WireToIntentCode;
typedef std::unordered_map<WirePtr, NodePtr, PtrHasher, PtrEqual> WireToNode;
typedef std::unordered_map<WirePtr, std::vector<WirePtr>, PtrHasher, PtrEqual>
    WireToWires;
typedef std::unordered_map<WirePtr, SitePinPtr, PtrHasher, PtrEqual>
    WireToSitePin;
typedef std::unordered_map<SitePinPtr, WirePtr, PtrHasher, PtrEqual>
    SitePinToWire;

class SitePin {
public:
  PTR_FACTORY(SitePin)
  PTR_RECYCLE(SitePin)

  SitePin(uint32_t name_) : name_(name_) {}

  ACCESSOR_RO(uint32_t, name)
};

class Wire {
public:
  PTR_FACTORY(Wire)
  PTR_RECYCLE(Wire)

  Wire(uint32_t name_) : name_(name_) {}

  ACCESSOR_RO(uint32_t, name)
  // ACCESSOR_RO(IntentCode, intentcode)

  static IntentCode intentcode(const std::string &wire_name,
                               const std::string &wire_type_name,
                               const std::string &tile_type_name);
};

class PipContainer {
public:
  typedef std::unordered_map<
      WirePtr, std::unordered_map<WirePtr, bool, PtrHasher, PtrEqual>,
      PtrHasher, PtrEqual>
      Type;

  PipContainer(Type container) : container(container) {}

  bool contains(WirePtr wire0, WirePtr wire1) const {
    return container.contains(wire0) && container.at(wire0).contains(wire1);
  }
  bool is_directional(WirePtr wire0, WirePtr wire1) const {
    assert(contains(wire0, wire1));
    return container.at(wire0).at(wire1);
  }

private:
  Type container;

  friend class TileType;
};

class SiteType {
public:
  PTR_FACTORY(SiteType)
  PTR_RECYCLE(SiteType)

  SiteType(uint32_t name_, const std::vector<SitePinPtr> &pins) : name_(name_) {
    for (size_t index = 0; index < pins.size(); index++) {
      pin_to_index_[pins[index]] = index;
    }
  }

  typedef std::unordered_map<SitePinPtr, uint32_t, PtrHasher, PtrEqual>
      PinToIndex;
  ACCESSOR_RO(uint32_t, name)
  ACCESSOR_RO(PinToIndex, pin_to_index)
  // There may be multiple sites with the same type insde a tile.
  // The same pin name in each site connects to various wire names.
  // SiteType and SiteTypeInst's represent such correspondence.
};

class SiteTypeInst {
public:
  PTR_FACTORY(SiteTypeInst)
  PTR_RECYCLE(SiteTypeInst)

  SiteTypeInst(SiteTypePtr type_, const std::vector<WirePtr> &wires)
      : type_(type_), index_to_wire_(wires) {}

  ACCESSOR_RO(SiteTypePtr, type)
  ACCESSOR_RO(std::vector<WirePtr>, index_to_wire)
  // see SiteType::pin_to_index
};

class TileType {
public:
  PTR_FACTORY(TileType)
  PTR_RECYCLE(TileType)

  TileType(DeviceResources::Device::Reader fpgaif_device,
           DeviceResources::Device::TileType::Reader fpgaif_tile_type,
           std::vector<std::pair<WirePtr, IntentCode>> wires_with_intentcode,
           PipContainer::Type pips_, bool is_INT_)
      : name_(fpgaif_tile_type.getName()), pips_(pips_), is_INT_(is_INT_),
        is_route_excluded_(false),
        wire_intentcode_(wires_with_intentcode.begin(),
                         wires_with_intentcode.end()) {
    for (auto [wire, _] : wires_with_intentcode) {
      wires_.push_back(wire);
    }
    build(fpgaif_device, fpgaif_tile_type);
  }

  const std::vector<WirePtr> &get_downhill_wires(WirePtr wire) const {
    static const std::vector<WirePtr> empty;
    if (wire_to_downhill_wires_.contains(wire))
      return wire_to_downhill_wires_.at(wire);
    else
      return empty;
  }

  const std::vector<WirePtr> &get_uphill_wires(WirePtr wire) const {
    static const std::vector<WirePtr> empty;
    if (wire_to_uphill_wires_.contains(wire))
      return wire_to_uphill_wires_.at(wire);
    else
      return empty;
  }

  ACCESSOR_RO(uint32_t, name)
  ACCESSOR_RO(std::vector<WirePtr>, wires)
  ACCESSOR_RO(PipContainer, pips)
  ACCESSOR_RO(bool, is_INT)
  ACCESSOR_RO(std::vector<SiteTypeInstPtr>,
              site_type_insts) // initialized in build()

  ACCESSOR_RW_BOOL(route_excluded)

  const IntentCode &intentcode(WirePtr wire) const {
    return wire_intentcode_.at(wire);
  }

private:
  WireToIntentCode wire_intentcode_; // initialized in constructor

  WireToWires wire_to_downhill_wires_; // initialized in build()
  WireToWires wire_to_uphill_wires_;   // initialized in build()

  void build(DeviceResources::Device::Reader fpgaif_device,
             DeviceResources::Device::TileType::Reader fpgaif_tile_type);

  friend class Tile;
  static std::unordered_map<uint32_t, SiteTypePtr> site_type_lookup;
};

class Site {
public:
  PTR_FACTORY(Site)
  PTR_RECYCLE(Site)

  Site(uint32_t name_, SiteTypePtr type_, SiteTypeInstPtr type_inst_)
      : name_(name_), type_(type_), type_inst_(type_inst_) {}

  ACCESSOR_RO(uint32_t, name)
  ACCESSOR_RO(SiteTypePtr, type)
  ACCESSOR_RO(SiteTypeInstPtr, type_inst)
};

class Tile {
public:
  PTR_FACTORY(Tile)
  PTR_RECYCLE(Tile)

  Tile(DeviceResources::Device::Reader fpgaif_device,
       DeviceResources::Device::Tile::Reader fpgaif_tile, TileTypePtr type_,
       std::string name);

  void attach(WirePtr wire, NodePtr node) { wire_to_node_[wire] = node; }

  ACCESSOR_RO(uint32_t, name)
  ACCESSOR_RO(TileTypePtr, type)
  ACCESSOR_RO(int, x)
  ACCESSOR_RO(int, y)
  ACCESSOR_RO(std::vector<SitePtr>, sites)
  ACCESSOR_RO(WireToNode, wire_to_node)
};

class Node {
public:
  PTR_FACTORY(Node)
  PTR_RECYCLE(Node)

  Node(uint32_t id_, std::vector<TileWire> &wires_, TileWire &begin_wire_,
       TileWire &end_wire_)
      : id_(id_), wires_(wires_), begin_wire_(begin_wire_),
        end_wire_(end_wire_) {
    base_tile_ = wires_.front().first;
    base_wire_ = wires_.front().second;
  }

  ACCESSOR_RO(uint32_t, id)
  ACCESSOR_RO(std::vector<TileWire>, wires)
  ACCESSOR_RO(TileWire, begin_wire)
  ACCESSOR_RO(TileWire, end_wire)

  const TilePtr &base_tile() const { return base_tile_; }
  const WirePtr &base_wire() const { return base_wire_; }
  const TilePtr end_tile() const;
  const IntentCode &intentcode() const {
    return base_tile()->type()->intentcode(base_wire());
  }

  std::vector<NodePtr> get_downhill_nodes() const;
  std::vector<NodePtr> get_uphill_nodes() const;

private:
  TilePtr base_tile_;
  WirePtr base_wire_;
  static void sort_and_uniquify(std::vector<NodePtr> &nodes);
};

class PipInst {
public:
  PTR_FACTORY(PipInst)
  PTR_RECYCLE(PipInst)

  PipInst(TilePtr tile_, WirePtr wire0_, WirePtr wire1_ = nullptr,
          bool is_directional_ = false)
      : tile_(tile_), wire0_(wire0_), wire1_(wire1_),
        is_directional_(is_directional_) {
    is_stub_ = false;
    is_fixed_ = false;
  }

  bool is_null_end() { return wire1_ == nullptr; }

  NodePtr node0() { return tile()->wire_to_node().at(wire0()); }
  NodePtr node1() { return tile()->wire_to_node().at(wire1()); }

  ACCESSOR_RO(TilePtr, tile)
  ACCESSOR_RO(WirePtr, wire0)
  ACCESSOR_RO(WirePtr, wire1)
  ACCESSOR_RO(bool, is_directional)

  ACCESSOR_RW_BOOL(stub)
  ACCESSOR_RW_BOOL(fixed)
};

class SitePinInst {
public:
  PTR_FACTORY(SitePinInst)
  PTR_RECYCLE(SitePinInst)

  SitePinInst(TilePtr tile_, SitePtr site_, SitePinPtr pin_)
      : tile_(tile_), site_(site_), pin_(pin_) {
    auto index = site()->type()->pin_to_index().at(pin_);
    wire_ = site()->type_inst()->index_to_wire().at(index);
    node_ = tile()->wire_to_node().at(wire_);
  }

  ACCESSOR_RO(TilePtr, tile)
  ACCESSOR_RO(SitePtr, site)
  ACCESSOR_RO(SitePinPtr, pin)

  ACCESSOR_RO(WirePtr, wire)
  ACCESSOR_RO(NodePtr, node)
};

class Device {
public:
  PTR_FACTORY(Device)
  PTR_RECYCLE(Device)

  Device(std::string device_filename = "xcvu3p.device")
      : device_file([device_filename]() {
          std::string ungzip_device_filename = device_filename + ".ungzip";
          ungzip(device_filename, ungzip_device_filename);
          return std::make_unique<MappedFile>(ungzip_device_filename);
        }()),
        stream([this]() {
          kj::ArrayPtr<const kj::byte> arr(
              reinterpret_cast<const kj::byte *>(this->device_file->data()),
              this->device_file->size());
          return kj::ArrayInputStream(arr);
        }()),
        message([this]() {
          capnp::ReaderOptions options;
          options.traversalLimitInWords = 3000000000;
          return capnp::InputStreamMessageReader(this->stream, options);
        }()) {
    build();
  }

  const std::string &name(uint32_t id) const { return str_list.at(id); }
  template <typename T> const std::string &name(T entity) const {
    return name(entity->name());
  }
  uint32_t id(const std::string &name) const { return str_to_id.at(name); }

private:
  void build();
  std::vector<std::string> str_list;
  std::unordered_map<std::string, uint32_t> str_to_id;

  std::unique_ptr<MappedFile> device_file;
  kj::ArrayInputStream stream;
  capnp::InputStreamMessageReader message;

public:
  typedef std::unordered_map<uint32_t, TilePtr> NameIdToTilePtr;
  typedef std::unordered_map<uint32_t, WirePtr> NameIdToWirePtr;
  ACCESSOR_RO(NameIdToTilePtr, tiles)
  ACCESSOR_RO(NameIdToWirePtr, wires)
  ACCESSOR_RO(std::vector<TileTypePtr>, tile_types)
  ACCESSOR_RO(std::vector<NodePtr>, nodes)
  std::unordered_map<uint32_t, std::set<uint32_t>> laguna_tiles_to_specwires;
  std::unordered_set<uint32_t> laguna_tiles;
  std::unordered_set<uint32_t> accessible_wires_same_col;
};

} // namespace device

#endif // DEVICE_H
