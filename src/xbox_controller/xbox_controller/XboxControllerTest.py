import time
import importlib
import inputs

def main():
    print("Listening for gamepad events...")
    while True:
        try:
            events = inputs.get_gamepad()
            for event in events:
                print(event.ev_type, event.code, event.state)
                if event.ev_type == 'Key' or event.ev_type == 'Absolute':
                    print('hey!')
        except Exception as e:
            print("Error:", str(e))
            time.sleep(5)
            importlib.reload(inputs)  # Re-import the devices module to reinitialize

            continue

if __name__ == "__main__":
    main()