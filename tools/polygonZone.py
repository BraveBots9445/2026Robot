from wpimath.geometry import Translation2d, Pose2d


class PolygonZone:
    _vertices: list[Translation2d]

    def __init__(self, vertices: list[Translation2d]):
        if len(vertices) < 3:
            return None

        self._vertices = vertices

    def containsTranslation(self, point: Translation2d) -> bool:
        x = point.X()
        y = point.Y()

        inside = False
        n = len(self._vertices)

        for i in range(n):
            j = (i - 1) % n

            xi = self._vertices[i].X()
            yi = self._vertices[i].Y()
            xj = self._vertices[j].X()
            yj = self._vertices[j].Y()

            intersects = ((yi > y) != (yj > y)) and (
                x < (xj - xi) * (y - yi) / (yj - yi + 1e-9) + xi
            )

            if intersects:
                inside = not inside

        return inside

    def containsPose(self, pose: Pose2d) -> bool:
        return self.containsTranslation(pose.translation())
