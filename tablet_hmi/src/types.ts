export interface DetectionObject {
  track_id?: number;
  class_name?: string;
  class_id?: number;
  confidence?: number;
  bbox?: number[];
  center_px?: number[];
  base_position_m?: { x: number; y: number; z: number };
  [key: string]: unknown;
}

export interface DetectionPayload {
  camera?: string;
  camera_id?: string;
  header_stamp?: number;
  motion_session?: string;
  objects?: DetectionObject[];
  [key: string]: unknown;
}

export interface Snapshot {
  schema_version?: number;
  sequence?: number;
  server_time?: number;
  gateway?: Record<string, any>;
  device?: { id: string; name: string; role: "operator" | "observer" };
  lease?: Record<string, any> | null;
  workbench?: Record<string, any>;
  robot?: Record<string, any>;
  detections?: Record<string, DetectionPayload>;
  media?: Record<string, any>;
}
