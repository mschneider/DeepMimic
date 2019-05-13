CD /D C:\Users\nikita\Anaconda3\Scripts\

CALL activate.bat C:\Users\nikita\Anaconda3\envs\ml-agents

CD C:\Users\nikita\Desktop\DeepMimic

python DeepMimic.py --arg_file args\kin_char_args.txt --character_file data/characters/generated.json
