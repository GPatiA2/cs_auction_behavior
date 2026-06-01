# auction_behavior

Distributed multi-robot task allocation behavior built on the [cs4home architecture](https://github.com/CoreSenseEU/cs4home_architecture), using a plugin-based auction algorithm and inter-agent communication via [ca_structure](../collective_awareness_structure).

## cs4home Architecture

`AuctionBehavior` is a **CognitiveModule** — it extends `cs4home_core::CognitiveModule`, which is itself a `CascadeLifecycleNode`. Following the cs4home pattern, `on_configure()` creates and wires two sub-components:

```
AuctionBehavior  (cs4home_core::CognitiveModule)
├── AuctionBehaviorAfferent  (cs4home_core::Afferent)
│     └── subscribes to self_localization/pose in ONDEMAND mode
└── AuctionBehaviorCore      (cs4home_core::Core)
      ├── ROS 2 action server  (as2_msgs/action/Auction)
      ├── Auction algorithm plugin (pluginlib)
      ├── Item evaluation plugin  (pluginlib)
      └── CAGatewayClient  (ca_structure) ← inter-agent channel
```

The lifecycle transitions (`configure` → `activate` → `deactivate`) propagate from the CognitiveModule down to both the Afferent and Core, keeping the component graph in sync.

```cpp
// auction_behavior.cpp — on_configure() wires the graph
afferent_ = std::make_shared<AuctionBehaviorAfferent>(self);
afferent_->configure();                          // subscribes to state topics

auto core = std::make_shared<AuctionBehaviorCore>(self);
core->set_afferent(afferent_);                   // Core can pull state from Afferent
core->configure();                               // loads plugins, registers CA gateway
core_ = core;
```

### Afferent: state access

`AuctionBehaviorAfferent` uses `cs4home_core::Afferent::ONDEMAND` mode so the Core can read the drone's latest pose at any point during execution:

```cpp
// Inside AuctionBehaviorCore::configure():
afferent_->set_mode(
  0,                               // topic index for self_localization/pose
  cs4home_core::Afferent::ONDEMAND,
  nullptr);                        // no immediate callback — polled on demand

// Later, when computing a bid:
auto pose_msg = afferent_->get_msg<geometry_msgs::msg::PoseStamped>("self_localization/pose");
```

### Core: action server + inter-agent comms

`AuctionBehaviorCore` creates a `ca_structure::CAGatewayClient` attached to the parent lifecycle node. The client registers for the inter-agent message types the auction needs:

```cpp
#include "ca_structure/ca_gateway_client.hpp"

ca_client_.register_module<as2_msgs::msg::StartAuction>(
  "auction_item_array", "auction_behavior",
  [this](const as2_msgs::msg::StartAuction & msg, const std::string & sender) {
    auction_plugin_->on_auction_items_received(msg.items, sender);
  });

ca_client_.register_module<as2_msgs::msg::Bid>(
  "bid", "auction_behavior",
  [this](const as2_msgs::msg::Bid & msg, const std::string & sender) {
    auction_plugin_->update(msg, sender);
  });
```

---

## Overview

Each drone runs an independent `AuctionBehavior` node. One drone acts as **auctioneer** — it receives an action goal listing the tasks and participants, broadcasts a `StartAuction` message via `ca_structure`, and kicks off the first bid. Every other drone acts as a **participant** — it receives the `StartAuction` message, computes its costs, and joins the bidding round. All bids travel over the inter-agent channel so no shared memory or central coordinator is required.

```
Drone A (auctioneer)                       Drone B (participant)
┌─────────────────────────┐               ┌─────────────────────────┐
│  AuctionBehavior        │               │  AuctionBehavior        │
│    plugin: greedy /     │  StartAuction │    plugin: greedy /     │
│           cbba          │──────────────►│           cbba          │
│                         │     Bid       │                         │
│                         │◄─────────────►│                         │
│  [converged → SUCCESS]  │               │  [converged → SUCCESS]  │
└─────────────────────────┘               └─────────────────────────┘
         ▲
    Action goal
    (tasks + bidders)
```

Task items (e.g. 2-D coordinates) are handled by a separate **item plugin** that computes each drone's cost for a task given its current state.

## Installation

### 1. Build the required packages

Both `cs4home_core` (architecture framework) and `ca_structure` (inter-agent communication) must be built first:

```bash
# cs4home architecture
mkdir -p ~/cs4home_ws/src
cd ~/cs4home_ws/src
git clone https://github.com/CoreSenseEU/cs4home_architecture.git
cd ~/cs4home_ws
colcon build --packages-select cs4home_core

# collective awareness / inter-agent messaging
cd ~/cs4home_ws/src
git clone <collective_awareness_structure-repository>
cd ~/cs4home_ws
colcon build --packages-select ca_msgs ca_structure
```

Build the required Aerostack2 message package:

```bash
mkdir -p ~/aerostack2_ws/src
cd ~/aerostack2_ws/src
git clone https://github.com/GPatiA2/aerostack2.git
cd ~/aerostack2_ws
colcon build --packages-select as2_msgs
```

Required packages:

| Package | Role |
|---|---|
| `cs4home_core` | CognitiveModule base class, Core, Afferent lifecycle framework |
| `ca_structure` | `CAGatewayClient` for inter-agent communication |
| `ca_msgs` | `InterAgentMessage`, `LocalGenericMessage`, `RegisterModule` service |
| `as2_msgs` | `Auction` action, `Bid`, `StartAuction`, `AuctionItem` message types |

### 2. Build this package

```bash
mkdir -p ~/cs_test_ws/src
cd ~/cs_test_ws/src
git clone <this-repository>
cd ~/cs_test_ws
source ~/cs4home_ws/install/setup.bash
source ~/aerostack2_ws/install/setup.bash
colcon build --packages-select auction_behavior
```

## AuctionBehavior node

The behavior server that manages the full auction lifecycle — plugin loading, auctioneer/participant role selection, bid convergence detection, and result publication.

### Running the node

```bash
ros2 launch auction_behavior auction_behavior_launch.py \
  namespace:=drone0 \
  plugin_name:=greedy_sequential
```

Or directly:

```bash
ros2 run auction_behavior auction_behavior_node --ros-args \
  -r __ns:=/drone0 \
  -p plugin_name:=greedy_sequential \
  -p state_component:='["self_localization/pose"]'
```

### Parameters

| Parameter | Default | Description |
|---|---|---|
| `plugin_name` | `greedy_sequential` | Algorithm plugin to load (`greedy_sequential` or `cbba`) |
| `state_component` | `[]` | List of state topics the item plugin needs (e.g. `self_localization/pose`) |
| `bundle_size` | `1` | Max tasks per drone (CBBA only; greedy always claims one) |

### Action interface

| Action | Type | Description |
|---|---|---|
| `AuctionBehavior` | `as2_msgs/action/Auction` | Main behavior action; goal carries item list, type, and participant namespaces |

**Goal fields:**

| Field | Type | Description |
|---|---|---|
| `name` | `string` | Auction identifier |
| `type` | `string` | Item plugin type (e.g. `coordinate_item`) |
| `elements` | `AuctionItem[]` | Task items with names and feature vectors |
| `bidders` | `string[]` | Namespace list of all participating drones |

**Result fields:**

| Field | Type | Description |
|---|---|---|
| `winners` | `string[]` | Winning drone namespace per task (same order as `elements`) |
| `elements` | `AuctionItem[]` | Full item list (mirrors goal for convenience) |

### Inter-agent messages (via ca_structure)

| Type key | ROS message | Direction | Description |
|---|---|---|---|
| `auction_item_array` | `as2_msgs/msg/StartAuction` | Auctioneer → Participants | Broadcasts the task list and participant roster |
| `bid` | `as2_msgs/msg/Bid` | All → All | Carries each drone's cost vector for all tasks |

### Data flow

```
Action goal (auctioneer)
        │
        ▼
  StartAuction ──(ca_structure)──► all participants
        │
        ▼
  Bid (own costs) ──(ca_structure)──► all peers
        │
        ▼
  on_bid_received → update() → solve_conflicts()
        │
        ▼
  check_convergence() == true
        │
        ▼
  get_result() → action SUCCESS
```

## Algorithm plugins

### greedy_sequential

Each drone computes its cost for every task and broadcasts a single bid immediately. Once every participant's bid has been received, per-item conflict resolution assigns each task to the lowest-cost drone (lexicographic tie-break on namespace). No iterative rounds are required.

**Config** (`plugins/greedy_sequential/config/default.yaml`):

```yaml
auction_behavior:
  ros__parameters:
    plugin_name: "greedy_sequential"
    bundle_size: 1
```

### cbba

Implements the Consensus-Based Bundle Algorithm (Choi et al., 2009). Each drone builds a bundle of up to `bundle_size` tasks greedily, broadcasts its full `(y, z)` state, and iteratively updates on peer bids until no state change occurs (convergence). Supports multi-task assignment per drone.

**Config** (`plugins/cbba/config/default.yaml`):

```yaml
auction_behavior:
  ros__parameters:
    plugin_name: "cbba"
    bundle_size: 1
```

## Item plugins

### coordinate_item

Represents a 2-D target coordinate. Cost equals the Euclidean distance from the drone's current XY position to the target. Requires `self_localization/pose` in `state_component`.

**AuctionItem layout:**

| Field | Content |
|---|---|
| `name` | Task identifier |
| `features` | `[x, y]` — target position in metres |
| `feature_names` | `["x", "y"]` |

**Config** (`plugins/coordinate_item/config/default.yaml`):

```yaml
auction_behavior:
  ros__parameters:
    item_plugin_type: "coordinate_item"
    state_component:
      - "self_localization/pose"
```

## Plugin API

### AuctionBehaviorPluginBase

| Method | Description |
|---|---|
| `initialize(node, client)` | Attach to a node and the CA gateway client |
| `on_auction_items_received(msg, agent_id)` | Called when items arrive; populates `auction_items_` |
| `compute_bid()` | Return a `Bid` message to send to peers |
| `update(bid, agent_id)` | Incorporate a peer's bid into local state |
| `check_convergence()` | Return `true` when the auction is settled |
| `get_result()` | Return the full global assignment (all items + winners) |
| `on_run()` | Periodic tick; override for retransmission logic |

### AuctionItemPluginBase

| Method | Description |
|---|---|
| `create(item_msg)` | Factory: produce a typed item from an `AuctionItem` message |
| `evaluate(state_interface)` | Compute this drone's cost for the item |
| `get_name()` | Return the task identifier |
| `get_item()` | Return the original `AuctionItem` message |
