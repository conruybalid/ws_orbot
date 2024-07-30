from inputs import get_gamepad

def main():
    print("Listening for gamepad events...")
    while True:
        events = get_gamepad()
        for event in events:
            print(event.ev_type, event.code, event.state)

if __name__ == "__main__":
    main()