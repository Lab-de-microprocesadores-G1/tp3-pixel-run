/********************************************************************************
  @file     App.c
  @brief    Application functions
  @author   N. Magliola, G. Davidov, F. Farall, J. Gaytán, L. Kammann, N. Trozzo
 *******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

// #include "superpower.h"
#include <stdint.h>
#include <stdbool.h>
#include "kernel/kernel.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define DISPLAY_SIZE	       	        8	// Display side number of digits (8x8)
#define RUNNER_POS_INIT                 0


enum 
{
    POS_X,
    POS_Y
};

//  UP-LEFT CORNER -----------> (+x)
//  .
//  .           SCREEN
//  .
//  v (+y)



/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/

/**
 * @brief   Updates a matrix of (DISPLAY_SIZE)x(DISPLAY_SIZE) RGBs (3 bytes) and
 *          calls the kernel method that updates the LEDs matrix.
 */
static void updateDisplay(void);

/**
 * @brief   Scrolls down the obstacles. Checks if there was a collision and acts
 *          accordingly. Calls updateDisplay() in the end.
 */
static void scrollObstacles(void);

/**
 * @brief   Generates a new obstacle line.
 */
static kernel_color_t * generateObstacleLine(void);

/**
 * @brief   Moves player to the right. Calls updateDisplay() in the end.
 */
static void moveRight(void);

/**
 * @brief   Moves player to the left. Calls updateDisplay() in the end.
 */
static void moveLeft(void);

/**
 * @brief   Toggles the variable gameContext.gamePaused.
 */
static void togglePause(void);

/**
 * @brief   Checks if there was a collision. If so, calls game-over animation
 *          resets the game and pauses it.
 */
static void checkCollision(void);

/**
 * @brief   Resets game values to initial ones.
 */
static void gameInit(void);

/**
 * @brief   Resets game values to initial ones.
 */
static void gameOverUpdate(void);


/*******************************************************************************
 * VARIABLES TYPES DEFINITIONS
 ******************************************************************************/

typedef struct
{
    uint16_t            score;                   
    uint8_t             runnerPos;                          // runner´s position (0 to DISPLAY_SIZE - 1)
    bool                gamePaused;                         // pause status
    difficulty_t        difficulty;                         // game difficulty
    uint8_t             tileOfBlocks;                       // last tile extracted from blocksPattern
    bool                blankTile;
    kernel_color_t      board[DISPLAY_SIZE][DISPLAY_SIZE];  // matrix containing colours of each led in the display
    bool                gameOverAnimation;
    uint8_t             gameOverIndex;
} pixelrun_context_t;


typedef enum
{
    DIFF_0,
    DIFF_1,
    DIFF_2,
    DIFF_3,
    DIFF_NUM
} difficulty_t;

typedef kernel_color_t pixelrun_block_matrix_t[7][DISPLAY_SIZE];

/*******************************************************************************
 * PRIVATE VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

static pixelrun_context_t       gameContext;                // Game context and data
static pixelrun_block_matrix_t  blocksPattern = 
{
    {KERNEL_BLUE, KERNEL_BLUE, KERNEL_BLUE, KERNEL_BLACK, KERNEL_BLACK, KERNEL_BLACK, KERNEL_BLACK, KERNEL_BLUE},
    {KERNEL_BLACK, KERNEL_BLACK, KERNEL_BLUE, KERNEL_BLUE, KERNEL_BLACK, KERNEL_BLACK, KERNEL_BLUE, KERNEL_BLUE},
    {KERNEL_BLACK, KERNEL_BLUE, KERNEL_BLUE, KERNEL_BLACK, KERNEL_BLUE, KERNEL_BLUE, KERNEL_BLACK, KERNEL_BLACK},
    {KERNEL_BLACK, KERNEL_BLACK, KERNEL_BLUE, KERNEL_BLACK, KERNEL_BLACK, KERNEL_BLUE, KERNEL_BLUE, KERNEL_BLUE},
    {KERNEL_BLUE, KERNEL_BLUE, KERNEL_BLUE, KERNEL_BLUE, KERNEL_BLACK, KERNEL_BLACK, KERNEL_BLACK, KERNEL_BLACK},
    {KERNEL_BLUE, KERNEL_BLACK, KERNEL_BLACK, KERNEL_BLUE, KERNEL_BLUE, KERNEL_BLACK, KERNEL_BLACK, KERNEL_BLUE},
    {KERNEL_BLACK, KERNEL_BLACK, KERNEL_BLUE, KERNEL_BLUE, KERNEL_BLUE, KERNEL_BLUE, KERNEL_BLACK, KERNEL_BLACK},
};          // Matrix to draw tiles of obstacles from

/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

/* Called once at the beginning of the program */
void appInit (void)
{
    kernelInit();
    gameInit();
}

/* Called repeatedly in an infinite loop */
void appRun (void)
{
    if (kernelIsEvent())
    {
        kernel_event_t ev = kernelGetNextEvent();
        switch (ev.id)
        {
            case KERNEL_TIMEOUT:
            { 
                if (gameContext.gameOverAnimation)
                {
                    gameOverUpdate();
                }
                else
                {
                    if(!gameContext.gamePaused) 
                        scrollObstacle();
                } 
            } break;
            case KERNEL_RIGHT: if(!gameContext.gamePaused && !gameContext.gameOverAnimation) moveRight(); break;
            case KERNEL_LEFT: if(!gameContext.gamePaused && !gameContext.gameOverAnimation) moveLeft(); break;
            case KERNEL_ENTER: if(!gameContext.gameOverAnimation) togglePause(); break;
        }
        
        checkCollision();
        updateDisplay();
    }
}
/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/
void gameInit(void)
{
    gameContext.runnerPos = RUNNER_POS_INIT;
    gameContext.difficulty = DIFF_0;
    gameContext.score = 0;
    gameContext.gamePaused = false;
    
    gameContext.tileOfBlocks = 3;
    gameContext.blankTile = true;
    
    for (uint8_t i=0; i<DISPLAY_SIZE; i++)
    {
        if (i % 2)
        {
            for (uint8_t j=0; j<DISPLAY_SIZE; j++)
            {
                gameContext.board[i][j] = blocksPattern[i/2][j];

            }
        }
        else
        {
            for (uint8_t j=0; j<DISPLAY_SIZE; j++)
            {
                gameContext.board[0][j] = KERNEL_BLACK;
            }
        }
    }

    gameContext.gameOverAnimation = false;
    gameContext.gameOverIndex = 0;
}

void togglePause(void)
{
    gameContext.gamePaused = !gameContext.gamePaused;
}

void moveLeft(void)
{
    if (gameContext.runnerPos < 0 )
    {
        gameContext.runnerPos--;
    }
}

void moveRight(void)
{
    if (gameContext.runnerPos < (DISPLAY_SIZE-1) )
    {
        gameContext.runnerPos++;
    }
}

void scrollObstacles(void)
{
    for (uint8_t i=1; i<DISPLAY_SIZE; i++)
    {
        for (uint8_t j=0; j<DISPLAY_SIZE; j++)
        {
            gameContext.board[i][j] = gameContext.board[i-1][j];
        }
    }

    if (gameContext.blankTile)
    {
        for (uint8_t j=0; j<DISPLAY_SIZE; j++)
        {
            gameContext.board[0][j] = KERNEL_BLACK;
        }
    }
    else
    {
        kernel_color_t * newObstaclesTile = generateObstacleLine();
        for (uint8_t j=0; j<DISPLAY_SIZE; j++)
        {
            gameContext.board[0][j] = newObstaclesTile[j];
        }
    }

    gameContext.blankTile = !gameContext.blankTile;
}

kernel_color_t * generateObstacleLine(void)
{
    return blocksPattern[gameContext.tileOfBlocks++];
}

void updateDisplay(void)
{
    kernelDisplay(gameContext.board);
}

void checkCollision(void)
{
    return (gameContext.board[DISPLAY_SIZE-1][gameContext.runnerPos] != KERNEL_BLACK); // checks color in runner's position
}

void gameOverUpdate(void)
{
    if (gameContext.gameOverAnimation < DISPLAY_SIZE*DISPLAY_SIZE)
    {
        // TODO kernelStartTimer(timer, 50, false);
        *((kernel_color_t*)gameContext.board + gameContext.gameOverIndex++) = KERNEL_RED;
    }
    else
    {
        gameInit();
    }
}
/*******************************************************************************
 ******************************************************************************/