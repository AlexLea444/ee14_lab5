if __name__ == "__main__":
    board = [[0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0],
             [0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0],
             [0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0]]

    output = ""
    for i in range(0, 9):
        for j in range(0, 9):
            if not j % 3:
                output = output + "| "
            output = output + F"{board[i][j]} "

        if (not i % 3):
            print(" ")
        print(output + "|")
        output = ""
