"""HTTP policy server for the robo-arm project."""

import argparse

import uvicorn
from fastapi import FastAPI
from pydantic import BaseModel, Field


class JointStatePayload(BaseModel):
    name: list[str] = Field(default_factory=list)
    position: list[float] = Field(default_factory=list)
    effort: list[float] = Field(default_factory=list)


class PredictRequest(BaseModel):
    image_jpeg_b64: str
    joint_state: JointStatePayload


class PredictResponse(BaseModel):
    action: list[float]


app = FastAPI()


@app.get("/healthz")
async def healthz():
    return {"ok": True}


@app.post("/predict", response_model=PredictResponse)
async def predict(request: PredictRequest):
    positions = request.joint_state.position
    torques = request.joint_state.effort

    new_action = [0.0, 0.0, 0.0, 0.0]

    return PredictResponse(action=new_action)


def main():
    parser = argparse.ArgumentParser(description="Run the robo-arm policy server")
    parser.add_argument("--host", default="0.0.0.0", help="Bind host (default: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=8000, help="HTTP port (default: 8000)")
    args = parser.parse_args()

    uvicorn.run(app, host=args.host, port=args.port)


if __name__ == "__main__":
    main()
