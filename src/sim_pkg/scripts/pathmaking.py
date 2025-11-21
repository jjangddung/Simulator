#!/usr/bin/env python3
# coding: utf-8

import rclpy
from rclpy.node import Node

import xml.etree.ElementTree as ET
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class GraphPathPublisher(Node):
    def __init__(self):
        super().__init__('graph_path_publisher')

        # GraphML 파일 경로 파라미터
        self.declare_parameter(
            'file_path',
            '/home/dongmin/bfmc/Simulator/src/sim_pkg/path/gercek2.graphml'
        )
        self.file_path = self.get_parameter('file_path').get_parameter_value().string_value

        # Path publisher
        self.path_pub = self.create_publisher(Path, 'track_path', 10)

        # Path를 한 번 만들어서 멤버 변수로 저장
        self.path_msg = self.build_path()

        if self.path_msg is None:
            self.get_logger().error("Path build failed. No publishing will happen.")
        else:
            self.get_logger().info(
                f"Path built with {len(self.path_msg.poses)} poses. "
                "Will publish periodically on 'track_path'."
            )
            # 🔁 0.5초마다 계속 퍼블리시 (RViz2, topic echo에서 확실히 보이게)
            self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        if self.path_msg is None:
            return
        # 현재 시간으로 헤더 갱신해서 퍼블리시
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path_msg)

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

        # True edge가 하나도 없다면 전체 edge로 대체 (디버깅용 안전장치)
        if not edges_true:
            self.get_logger().warn("No true edges found. Using all edges instead.")
            edges_true = [(s, t, True) for (s, t, _) in edges_all]

        # adjacency list 구성
        adj = {}
        for src, tgt, _ in edges_true:
            adj.setdefault(src, []).append(tgt)
            adj.setdefault(tgt, []).append(src)

        if not adj:
            self.get_logger().error("Adjacency list is empty. Cannot build path.")
            return None

        # ✅ 시작 노드를 '263'으로 강제
        start_node = "263"
        if start_node not in adj:
            self.get_logger().warn(
                f"Start node 263 not found in adjacency. "
                f"Available sample node: {list(adj.keys())[:5]}"
            )
            # 263이 진짜 없다면 fallback: degree 1 노드 찾기
            for nid, neighs in adj.items():
                if len(neighs) == 1:
                    start_node = nid
                    self.get_logger().warn(f"Fallback start node set to {start_node}")
                    break
            else:
                # 그것도 없으면 그냥 가장 작은 id
                start_node = sorted(adj.keys(), key=lambda x: int(x))[0]
                self.get_logger().warn(f"Fallback start node (min id) set to {start_node}")
        else:
            self.get_logger().info("Using node 263 as start node.")

        # 그래프를 따라가며 순서 있는 노드 리스트 만들기
        ordered_nodes = []
        visited = set()
        current = start_node
        prev = None

        while current is not None:
            ordered_nodes.append(current)
            visited.add(current)

            next_node = None
            for n in adj[current]:
                if n != prev and n not in visited:
                    next_node = n
                    break

            prev = current
            current = next_node

        self.get_logger().info(f"Built ordered node sequence of length {len(ordered_nodes)}")

        # nav_msgs/Path 생성
        path_msg = Path()
        # RViz2에서 Fixed Frame을 'map'으로 쓸 거라고 가정
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for nid in ordered_nodes:
            if nid not in node_pos:
                self.get_logger().warn(f"Node id {nid} has no position. Skipping.")
                continue
            x, y = node_pos[nid]
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0  # yaw는 일단 0
            path_msg.poses.append(pose)

        if not path_msg.poses:
            self.get_logger().error("Path has no poses. Check GraphML/filters.")
            return None

        return path_msg

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

    def extract_edges_data(self, root):
        edges_data = []
        edges_data_true = []
        ns = {'g': 'http://graphml.graphdrawing.org/xmlns'}

        for edge in root.findall(".//g:edge", ns):
            source_id = edge.get('source')
            target_id = edge.get('target')

            data_d2 = edge.find("g:data[@key='d2']", ns)
            d2_value = data_d2 is not None and data_d2.text == 'True'

            source_id_int = int(source_id)
            target_id_int = int(target_id)

            # 🔽 여기 범위는 기존 코드 그대로 유지
            ranges = [(469, 479), (409, 425), (386, 398), (357, 365), (259, 281)]
            if (any(start <= source_id_int <= end for start, end in ranges) and
                    any(start <= target_id_int <= end for start, end in ranges)):
                d2_value = True
                edges_data_true.append((source_id, target_id, d2_value))

            edges_data.append((source_id, target_id, d2_value))

        return edges_data, edges_data_true


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
