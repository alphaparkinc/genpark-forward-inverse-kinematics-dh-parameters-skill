"""MCP stdio server for Forward and Inverse Kinematics."""
import sys
import json
import math

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

from client import KinematicsEngine

def handle_rpc(request):
    req_id = request.get("id")
    method = request.get("method")
    params = request.get("params", {})

    if method == "tools/list":
        return {
            "jsonrpc": "2.0",
            "id": req_id,
            "result": {
                "tools": [
                    {
                        "name": "forward_kinematics_dh",
                        "description": "Compute joint positions and final end-effector pose from DH parameters",
                        "inputSchema": {
                            "type": "object",
                            "properties": {
                                "dh_params": {
                                    "type": "array",
                                    "items": {
                                        "type": "array",
                                        "items": {"type": "number"},
                                        "minItems": 4,
                                        "maxItems": 4
                                    },
                                    "description": "List of [theta, d, a, alpha] tuples"
                                }
                            },
                            "required": ["dh_params"]
                        }
                    },
                    {
                        "name": "inverse_kinematics_2link",
                        "description": "Compute joint angles (theta1, theta2) to reach planar target position (x, y)",
                        "inputSchema": {
                            "type": "object",
                            "properties": {
                                "l1": {"type": "number", "description": "Length of link 1"},
                                "l2": {"type": "number", "description": "Length of link 2"},
                                "target_x": {"type": "number", "description": "Target X coordinate"},
                                "target_y": {"type": "number", "description": "Target Y coordinate"}
                            },
                            "required": ["l1", "l2", "target_x", "target_y"]
                        }
                    }
                ]
            }
        }
    elif method == "tools/call":
        name = params.get("name")
        args = params.get("arguments", {})
        if name == "forward_kinematics_dh":
            dh_params = args.get("dh_params", [])
            positions, T = KinematicsEngine.forward_kinematics_dh(dh_params)
            return {"jsonrpc": "2.0", "id": req_id, "result": {"positions": positions, "transform": T}}
        elif name == "inverse_kinematics_2link":
            l1 = float(args.get("l1", 1.0))
            l2 = float(args.get("l2", 1.0))
            tx = float(args.get("target_x", 0.0))
            ty = float(args.get("target_y", 0.0))
            res = KinematicsEngine.inverse_kinematics_2link_planar(l1, l2, tx, ty)
            if res is None:
                return {"jsonrpc": "2.0", "id": req_id, "result": {"error": "Target unreachable", "angles": None}}
            return {"jsonrpc": "2.0", "id": req_id, "result": {"theta1_rad": res[0], "theta2_rad": res[1], "theta1_deg": math.degrees(res[0]), "theta2_deg": math.degrees(res[1])}}
        return {"jsonrpc": "2.0", "id": req_id, "error": {"code": -32601, "message": f"Method {name} not found"}}
    return {"jsonrpc": "2.0", "id": req_id, "error": {"code": -32600, "message": "Invalid request"}}

def main():
    for line in sys.stdin:
        if not line.strip():
            continue
        try:
            req = json.loads(line)
            res = handle_rpc(req)
            sys.stdout.write(json.dumps(res) + "\n")
            sys.stdout.flush()
        except Exception as e:
            sys.stdout.write(json.dumps({"jsonrpc": "2.0", "error": {"code": -32700, "message": str(e)}}) + "\n")
            sys.stdout.flush()

if __name__ == "__main__":
    main()
