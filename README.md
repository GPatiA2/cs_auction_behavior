# auction_behavior

Distributed multi-robot task allocation behavior for multi-agent systems built with [Aerostack2](https://github.com/GPatiA2/aerostack2), using a plugin-based auction architecture and inter-agent communication via [as2_ca](https://github.com/CoreSenseEU/collective_awareness_structure).

## Overview

Each drone runs an independent `AuctionBehavior` node. One drone acts as **auctioneer** — it receives an action goal listing the tasks and participants, broadcasts a `StartAuction` message via `as2_ca`, and kicks off the first bid. Every other drone acts as a **participant** — it receives the `StartAuction` message, computes its costs, and joins the bidding round. All bids travel over the inter-agent channel so no shared memory or central coordinator is required.

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

```bash
cd ~/cs_test_ws/src
git clone <this-repository>
cd ~/cs_test_ws
source ~/aerostack2_ws/install/setup.bash
colcon build --packages-select auction_behavior
```

Dependencies (provided by Aerostack2):

- `as2_core`, `as2_behavior`, `as2_msgs`, `as2_ca`
- `pluginlib`, `rclcpp`, `geometry_msgs`

## AuctionBehavior node

The behavior server that manages the full auction lifecycle — plugin loading, auctioneer/participant role selection, bid convergence detection, and result publication to the knowledge base.

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
| `name` | `string` | Auction identifier (stored in KB) |
| `type` | `string` | Item plugin type (e.g. `coordinate_item`) |
| `elements` | `AuctionItem[]` | Task items with names and feature vectors |
| `bidders` | `string[]` | Namespace list of all participating drones |

**Result fields:**

| Field | Type | Description |
|---|---|---|
| `winners` | `string[]` | Winning drone namespace per task (same order as `elements`) |
| `elements` | `AuctionItem[]` | Full item list (mirrors goal for convenience) |

### Inter-agent messages (via as2_ca)

| Type key | ROS message | Direction | Description |
|---|---|---|---|
| `auction_item_array` | `as2_msgs/msg/StartAuction` | Auctioneer → Participants | Broadcasts the task list and participant roster |
| `bid` | `as2_msgs/msg/Bid` | All → All | Carries each drone's cost vector for all tasks |

### Data flow

```
Action goal (auctioneer)
        │
        ▼
  StartAuction ──(as2_ca)──► all participants
        │
        ▼
  Bid (own costs) ──(as2_ca)──► all peers
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
