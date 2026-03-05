def policy(images_b64: list[str], joints: list[float], task: str) -> list[float]:
    del images_b64, joints, task
    return [0.0, 0.0, 0.0, 0.0]
