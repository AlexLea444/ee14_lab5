from time import sleep

insert_mode = 0
insert_index = 0

def print_board(board, cursor = [-1,-1]):
    sleep(0.5)
    output = ""
    for i in range(0, 9):
        for j in range(0, 9):
            if not j % 3:
                output = output + "| "
            if [i,j] == cursor:
                output = output + "_ "
            else:
                output = output + F"{board[i][j]} "

        if (not i % 3):
            print(" ")
        print(output + "|")
        output = ""
    sleep(0.5)
    for i in range(0, 9):
        for j in range(0, 9):
            if not j % 3:
                output = output + "| "
            output = output + F"{board[i][j]} "
        if (not i % 3):
            print(" ")
        print(output + "|")
        output = ""


if __name__ == "__main__":
    board = [[0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0],
             [0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0],
             [0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0]]
    cursor = [0, 0]
    inserts = "_123456789"
    while (1):
        print_board(board, cursor)
