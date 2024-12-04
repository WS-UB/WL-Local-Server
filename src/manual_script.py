import os

def main():
    while True:
        # Display the menu
        print("\n---------------------------------------------------------------------------------- WL-LOCAL-SERVER INTERACTIVE TOOL ----------------------------------------------------------------------------------")
        print("Choose an option:")
        print("1. Upload new data to MinIO")
        print("2. Update latest location to map")
        print("3. Exit")
        print("--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------")

        # Get user input
        choice = input("Enter your choice: ")

        if choice == '1':
            # Run the data_processing.py script
            os.system("python3 data_processing.py")
        elif choice == '2':
            # Run the sync_minio_and_mqtt.py script
            os.system("python3 sync_minio_and_mqtt.py")
        elif choice == '3':
            # Exit the program
            print("Exiting program...")
            break
        else:
            print("Invalid choice, please enter 1, 2, or 3.")

if __name__ == "__main__":
    main()
