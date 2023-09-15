class Site:
    def __init__(
            self,
            name: str = "000",
            floor: int = 0,
            position: tuple = (0, 0, 0),
            orientation: tuple =  (0, 0, 0, 1),
            state: int = 0
            ):
        self.name = name
        self.floor = floor
        self.position = position
        self.orientation = orientation
        self.state = state
