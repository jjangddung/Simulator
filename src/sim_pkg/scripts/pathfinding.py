#!/usr/bin/env python3
# coding: utf-8

import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


class GraphDataPublisher(Node):
    def __init__(self):
        super().__init__('graph_data_publisher')

        # === 파라미터 선언 ===
        self.declare_parameter(
            'file_path',
            '/home/dongmin/bfmc/Simulator/src/sim_pkg/path/gercek2.graphml'
        )
        self.declare_parameter('frame_id', 'map')  # RViz2에서 쓸 좌표계 이름

        self.file_path = self.get_parameter('file_path').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        # === Publisher들 ===
        self.string_pub = self.create_publisher(String, 'graph_data', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'graph_markers', 10)

        # 그래프 데이터 캐시용 변수
        self.nodes_data = []       # [(node_id, x, y), ...]
        self.edges_data_true = []  # [(source_id, target_id, True), ...]

        # 그래프 파일을 한 번만 읽어서 메모리에 저장
        self.load_graphml()

        # RViz2에서 안 놓치게 주기적으로 publish
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1초마다 발행

    # ===== GraphML 로딩 =====
    def load_graphml(self):
        try:
            tree = ET.parse(self.file_path)
            root = tree.getroot()
        except Exception as e:
            self.get_logger().error(f"Failed to parse GraphML file: {e}")
            return

        self.nodes_data = self.extract_nodes_data(root)
        _, self.edges_data_true = self.extract_edges_data(root)

        self.get_logger().info(
            f"Loaded GraphML: {len(self.nodes_data)} nodes, "
            f"{len(self.edges_data_true)} important edges"
        )

    # ===== 주기적으로 호출됨 (String + MarkerArray publish) =====
    def timer_callback(self):
        # 1) String으로도 한번 발행 (디버깅/다른 노드용)
        msg = String()
        msg.data = str(self.edges_data_true)
        self.string_pub.publish(msg)

        # 2) RViz2용 MarkerArray 발행
        markers = self.make_markers(self.nodes_data, self.edges_data_true)
        self.marker_pub.publish(markers)

    # ===== 노드 데이터 추출 =====
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
                    d0 = float(data.text)  # x
                elif key == 'd1':
                    d1 = float(data.text)  # y
            if d0 is not None and d1 is not None:
                nodes_data.append((node_id, d0, d1))
        return nodes_data

    # ===== 엣지 데이터 추출 =====
    def extract_edges_data(self, root):
        edges_data = []
        edges_data_true = []
        ns = {'g': 'http://graphml.graphdrawing.org/xmlns'}

        ranges = [(469, 479), (409, 425), (386, 398), (357, 365), (259, 281)]

        for edge in root.findall(".//g:edge", ns):
            source_id = edge.get('source')
            target_id = edge.get('target')

            data_d2 = edge.find("g:data[@key='d2']", ns)
            d2_value = data_d2 is not None and data_d2.text == 'True'

            source_id_int = int(source_id)
            target_id_int = int(target_id)

            # 특정 구간에 들어가면 강제로 True 처리
            if (any(start <= source_id_int <= end for start, end in ranges) and
                    any(start <= target_id_int <= end for start, end in ranges)):
                d2_value = True
                edges_data_true.append((source_id, target_id, d2_value))

            edges_data.append((source_id, target_id, d2_value))

        return edges_data, edges_data_true

    # ===== RViz2용 MarkerArray 생성 =====
    def make_markers(self, nodes_data, edges_data_true):
        marker_array = MarkerArray()

        node_dict = {nid: (x, y) for nid, x, y in nodes_data}
        now = self.get_clock().now().to_msg()

        # --- 1) 노드 표시용 POINTS 마커 ---
        node_marker = Marker()
        node_marker.header.frame_id = self.frame_id
        node_marker.header.stamp = now
        node_marker.ns = 'graph_nodes'
        node_marker.id = 0
        node_marker.type = Marker.POINTS
        node_marker.action = Marker.ADD

        # 점의 크기
        node_marker.scale.x = 0.2
        node_marker.scale.y = 0.2

        # 색 (초록색)
        node_marker.color.a = 1.0
        node_marker.color.r = 0.0
        node_marker.color.g = 1.0
        node_marker.color.b = 0.0

        for nid, (x, y) in node_dict.items():
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.0
            node_marker.points.append(p)

        marker_array.markers.append(node_marker)

        # --- 2) True 엣지용 LINE_LIST 마커 ---
        edge_marker = Marker()
        edge_marker.header.frame_id = self.frame_id
        edge_marker.header.stamp = now
        edge_marker.ns = 'graph_edges_true'
        edge_marker.id = 1
        edge_marker.type = Marker.LINE_LIST
        edge_marker.action = Marker.ADD

        edge_marker.scale.x = 0.1  # 선 두께

        # 색 (빨간색)
        edge_marker.color.a = 1.0
        edge_marker.color.r = 1.0
        edge_marker.color.g = 0.0
        edge_marker.color.b = 0.0

        for src_id, tgt_id, flag in edges_data_true:
            if src_id in node_dict and tgt_id in node_dict:
                sx, sy = node_dict[src_id]
                tx, ty = node_dict[tgt_id]

                ps = Point()
                ps.x = sx
                ps.y = sy
                ps.z = 0.0

                pt = Point()
                pt.x = tx
                pt.y = ty
                pt.z = 0.0

                edge_marker.points.append(ps)
                edge_marker.points.append(pt)

        marker_array.markers.append(edge_marker)

        return marker_array


def main(args=None):
    rclpy.init(args=args)
    node = GraphDataPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
