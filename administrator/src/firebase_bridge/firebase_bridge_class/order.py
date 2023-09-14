class Order:
    def __init__(
        self,
        missionType: str = "",
        note: str = "",
        random_password: int = 0,
        recipient: str = "",
        recipient_location: str = "",
        sender: str = "",
        sender_location: str = "",
        state: int = -1,
    ):
        self.missionType = missionType
        self.note = note
        self.random_password = random_password
        self.recipient = recipient
        self.recipient_locatioin = recipient_location
        self.sender = sender
        self.sender_location = sender_location
        self.state = state

    def from_dict(self, source):
        order = Order(
            missionType=source["missionType"],
            note=source["note"],
            random_password=source["random_password"],
            sender=source["sender"],
            recipient=source["recipient"],
            recipient_location=source["recipient_location"],
            sender_location=source["sender_location"],
            state=source["state"],
        )
        return order

    def to_dict(self):
        order = {
            "missionType": self.missionType,
            "note": self.note,
            "ramdom_password": self.random_password,
            "sender": self.sender,
            "recipient": self.recipient,
            "sender_location": self.sender_location,
            "recipient_location": self.recipient_locatioin,
            "state": self.state,
        }
        return order

    def __repr__(self):
        return (
            "mission: "
            + self.missionType
            + "\n"
            + self.sender
            + " send to "
            + self.recipient
            + "\n"
            + "from "
            + self.sender_location
            + " to "
            + self.recipient_locatioin
        )
