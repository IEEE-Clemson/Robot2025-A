from math import ceil

#Find the end of the song
song =  '0 C5 1 43;6 A#4 1 43;8 A4 1 43;14 A4 1 43;16 A#4 1 43;22 A#4 1 43;24 A4 1 43;0 E5 1 43;8 F5 1 43;14 F5 1 43;16 E5 1 43;22 E5 1 43;24 F5 1 43;0 C4 1 43;2 D4 1 43;4 E4 1 43;8 F4 1 43;10 G4 1 43;12 A4 1 43;16 G4 1 43;18 A4 1 43;20 A#4 1 43;6 C4 1 43;14 F4 1 43;22 G4 1 43;26 A#4 1 43;28 C5 1 43;30 F4 1 43;0 C6 2 43;6 A#5 1 43;8 A5 2 43;14 A5 1 43;16 A#5 2 43;22 A#5 1 43;24 A5 2 43;24 C5 1 43;30 F#5 1 43;32 C5 1 43;38 A#4 1 43;40 A4 1 43;46 A4 1 43;48 A#4 1 43;54 A#4 1 43;56 A4 1 43;32 E5 1 43;38 E5 1 43;40 F5 1 43;46 F5 1 43;48 E5 1 43;54 E5 1 43;32 C4 1 43;34 D4 1 43;36 E4 1 43;40 F4 1 43;42 G4 1 43;44 A4 1 43;48 G4 1 43;50 A4 1 43;52 A#4 1 43;38 C4 1 43;46 F4 1 43;54 G4 1 43;32 C6 2 43;38 A#5 1 43;40 A5 2 43;46 A5 1 43;48 A#5 2 43;54 G5 2 43;56 F5 2 43;6 E5 1 43;60 D6 4 43;60 F5 4 43;60 C6 4 43'
notes = []

end = 0
splitSong = song.split(";")
for note in splitSong:
    snote = note.split(" ")
    testEnd = round(float(snote[0])) + ceil(float(snote[2]))
    if (testEnd > end):
        end = testEnd
        
#Create empty song structure
while (end > len(notes)):
    notes.append(None)

#Populate song structure with the notes
for note in splitSong:
    snote = note.split(" ")
    beat = round(float(snote[0]));
    
    if (notes[beat] == None):
        notes[beat] = []
    notes[beat].append([snote[1],ceil(float(snote[2]))]) #Note, Duration

# Chords need multiple buzzers to play, only use first note since we only have one buzzer 
filtered_notes = []
for n in notes:
    pass