#!/usr/bin/env python3
# coding: utf-8

import math
import heapq
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class GraphPathPublisher(Node):
    def __init__(self):
        super().__init__('graph_path_publisher')

        # ===== 파라미터 =====
        self.declare_parameter(
            'file_path',
            '/home/dongmin/bfmc/Simulator/src/sim_pkg/path/fixed2.graphml'
        )
        self.declare_parameter('start_node', '263')
        self.declare_parameter('goal_node', '225')  # 필요에 따라 바꿔 쓰기

        self.file_path = self.get_parameter(
            'file_path').get_parameter_value().string_value
        self.start_node = self.get_parameter(
            'start_node').get_parameter_value().string_value
        self.goal_node = self.get_parameter(
            'goal_node').get_parameter_value().string_value

        # Path publisher
        self.path_pub = self.create_publisher(Path, 'track_path', 10)

        # Path를 한 번 만들어서 멤버 변수로 저장
        self.path_msg = self.build_path()

        if self.path_msg is None:
            self.get_logger().error("Path build failed. No publishing will happen.")
        else:
            self.get_logger().info(
                f"Path built with {len(self.path_msg.poses)} poses "
                f"from {self.start_node} to {self.goal_node}. "
                "Will publish periodically on 'track_path'."
            )
            # 🔁 0.5초마다 계속 퍼블리시
            self.timer = self.create_timer(0.5, self.timer_callback)

    # ---------------- 타이머: Path 주기적으로 발행 ----------------
    def timer_callback(self):
        if self.path_msg is None:
            return
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path_msg)

    # ---------------- Path 생성 ----------------
    def build_path(self):
        # GraphML 읽기
        try:
            tree = ET.parse(self.file_path)
            root = tree.getroot()
        except Exception as e:
            self.get_logger().error(f"Failed to parse GraphML file: {e}")
            return None

        nodes_data = self.extract_nodes_data(root)          # [(id, x, y), ...]
        edges_all, edges_true = self.extract_edges_data(root)

        # id -> (x, y) 매핑
        node_pos = {nid: (x, y) for nid, x, y in nodes_data}
        if not node_pos:
            self.get_logger().error("No nodes parsed from GraphML.")
            return None

        # 🔴 d2는 지금은 무시하고, 그래프 전체 edge 사용
        edges_use = edges_all
        if not edges_use:
            self.get_logger().error("No edges parsed from GraphML.")
            return None

        # adjacency list 구성 (무향 그래프)
        adj = {}
        for src, tgt, _ in edges_use:
            adj.setdefault(src, []).append(tgt)
            adj.setdefault(tgt, []).append(src)

        if not adj:
            self.get_logger().error("Adjacency list is empty. Cannot build path.")
            return None

        # 시작/목표 노드 존재 여부 체크
        if self.start_node not in adj:
            self.get_logger().error(
                f"Start node {self.start_node} not found in adjacency."
            )
            # 디버깅용으로 몇 개만 찍어보기
            sample_keys = list(adj.keys())[:10]
            self.get_logger().info(f"Adjacency sample keys: {sample_keys}")
            return None

        if self.goal_node not in adj:
            self.get_logger().error(
                f"Goal node {self.goal_node} not found in adjacency."
            )
            sample_keys = list(adj.keys())[:10]
            self.get_logger().info(f"Adjacency sample keys: {sample_keys}")
            return None

        self.get_logger().info(
            f"Running shortest path from {self.start_node} to {self.goal_node}."
        )

        # ===== Dijkstra (edge weight=1, 사실상 BFS) =====
        node_path = self.shortest_path(adj, self.start_node, self.goal_node)
        if node_path is None or len(node_path) == 0:
            self.get_logger().error(
                f"No path found from {self.start_node} to {self.goal_node}."
            )
            return None

        self.get_logger().info(
            f"Shortest path length (nodes) = {len(node_path)}"
        )

        # ===== nav_msgs/Path 생성 =====
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        # yaw 계산 위해 좌표 배열 만들기
        coords = []
        for nid in node_path:
            if nid not in node_pos:
                self.get_logger().warn(f"Node id {nid} has no position. Skipping.")
                continue
            coords.append((nid, *node_pos[nid]))  # (id, x, y)

        if not coords:
            self.get_logger().error("No valid nodes with positions found on path.")
            return None

        # 각 세그먼트마다 yaw 계산 (마지막 노드는 이전 yaw 재사용)
        yaws = []
        for i in range(len(coords) - 1):
            _, x1, y1 = coords[i]
            _, x2, y2 = coords[i + 1]
            dx = x2 - x1
            dy = y2 - y1
            yaw = math.atan2(dy, dx)
            yaws.append(yaw)
        if yaws:
            yaws.append(yaws[-1])  # 마지막 노드용 yaw
        else:
            yaws.append(0.0)

        for (nid, x, y), yaw in zip(coords, yaws):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            # yaw → quaternion (z,w)
            qz = math.sin(yaw / 2.0)
            qw = math.cos(yaw / 2.0)
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw

            path_msg.poses.append(pose)

        if not path_msg.poses:
            self.get_logger().error("Path has no poses after building. Check GraphML.")
            return None

        return path_msg

    # ---------------- 노드 파싱 ----------------
    def extract_nodes_data(self, root):
        nodes_data = []
        ns = {'g': 'http://graphml.graphdrawing.org/xmlns'}

        for node in root.findall(".//g:node", ns):
            node_id = node.get('id')
            d0 = None
            d1 = None
            for data in node:
                key = data.get('key')
                if key == 'd0':
                    d0 = float(data.text)
                elif key == 'd1':
                    d1 = float(data.text)
            if d0 is not None and d1 is not None:
                nodes_data.append((node_id, d0, d1))
        return nodes_data

    # ---------------- 엣지 파싱 ----------------
    def extract_edges_data(self, root):
        """
        GraphML에서 edge 전체를 읽고:
        - d2 값은 일단 읽어두기만 하고, path 생성에는 사용하지 않는다.
        """
        edges_data = []
        edges_data_true = []
        ns = {'g': 'http://graphml.graphdrawing.org/xmlns'}

        for edge in root.findall(".//g:edge", ns):
            source_id = edge.get('source')
            target_id = edge.get('target')

            data_d2 = edge.find("g:data[@key='d2']", ns)
            d2_value = (data_d2 is not None and data_d2.text == 'True')

            if d2_value:
                edges_data_true.append((source_id, target_id, d2_value))

            edges_data.append((source_id, target_id, d2_value))

        return edges_data, edges_data_true

    # ---------------- Dijkstra: 최단 노드 시퀀스 ----------------
    def shortest_path(self, adj, start, goal):
        """
        adj: {node_id: [neighbor_id, ...]}
        start, goal: node_id (string)
        return: [start, ..., goal] 혹은 None
        """
        dist = {nid: float('inf') for nid in adj.keys()}
        prev = {nid: None for nid in adj.keys()}
        dist[start] = 0.0

        pq = [(0.0, start)]  # (cost, node_id)

        while pq:
            cur_d, u = heapq.heappop(pq)
            if cur_d > dist[u]:
                continue
            if u == goal:
                break

            for v in adj.get(u, []):
                nd = cur_d + 1.0   # 모든 edge 가중치 1
                if nd < dist[v]:
                    dist[v] = nd
                    prev[v] = u
                    heapq.heappush(pq, (nd, v))

        if dist[goal] == float('inf'):
            return None

        path_ids = []
        cur = goal
        while cur is not None:
            path_ids.append(cur)
            cur = prev[cur]
        path_ids.reverse()
        return path_ids


def main(args=None):
    rclpy.init(args=args)
    node = GraphPathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
