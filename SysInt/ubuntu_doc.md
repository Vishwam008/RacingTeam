## Adding a new user:
`# adduser USRNM`

Output: `Full Name[]: ` Add data if needed or press Enter to give default values

### To give root priveleges
`visudo` then go to user privelege specification and write `NEWUSR ALL=(ALL:ALL) ALL` [like root]

`su NEWUSR` to change user

`sudo -i` to change to root




## Installing software
`apt-get update && apt-get install -y nano && rm -rf /var/lib/apt/lists/` to install nano

`sudo apt install curl` to install curl


## Adding to path
**Temporary:** `export PATH="/Directory1:$PATH"` where directory1 is the dir you want to add to path

**Permanent:** Add the command to .bashrc

