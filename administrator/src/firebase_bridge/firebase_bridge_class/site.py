class Site:
    def __init__(
        self,
        floor: int = 0,
        position: tuple = (0, 0, 0),
        orientation: tuple = (0, 0, 0, 1),
        state: int = 0,
    ):
        self.floor = floor
        self.position = position
        self.orientation = orientation
        self.state = state

    def from_dict(self, source):
        site = Site(
            floor=source["floor"],
            position=tuple(
                [
                    source["position"]["x"],
                    source["position"]["y"],
                    source["position"]["z"],
                ]
            ),
            orientation=tuple(
                [
                    source["orientation"]["x"],
                    source["orientation"]["y"],
                    source["orientation"]["z"],
                    source["orientation"]["w"],
                ]
            ),
            state=0,
        )
        return site

    def to_dict(self):
        pass

    def __repr__(self):
        pass
