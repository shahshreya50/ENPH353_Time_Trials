In order to use randomly generated clues with updated collect_images.py update plate_generator.py (in 2025_competition/enph353/enph353_gazebo/scripts/plate_generator.py) with the following:
ADD:
def loadCrimesProfileTraining():
    print("Generating clues ...")
    
    key_list = ['SIZE', 'VICTIM', 'CRIME', 'TIME', 'PLACE', 'MOTIVE', 'WEAPON', 'BANDIT']
    value_list = []
    for i in range(8):
        value_list.append(randomStringGen())
    
    clues = {}

    with open(SCRIPT_PATH + "clues.csv", 'w') as plates_file:
        csvwriter = csv.writer(plates_file)

        for (key, value) in zip(key_list, value_list):
            clues[key] = value.upper()

            # save it to plates
            csvwriter.writerow([key, value.upper()])

    return clues


def randomStringGen():
    characters = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789"
    word_len = random.randint(4,12)
    char_list = []
    for i in range(word_len):
        char_list.append(characters[random.randint(0,35)])
    return ''.join(char_list)

REPLACE:
clues = loadCrimesProfileCompetition()
WITH
clues = loadCrimesProfileTraining()
