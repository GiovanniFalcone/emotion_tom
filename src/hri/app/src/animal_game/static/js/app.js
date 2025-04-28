// List of possible cards
const baseCards = ['goose', 'koala', 'bird', 'tiger', 'panda', 'pelican', 'penguin', 'walrus', 'flamingo', 'shark', 'horse', 'duck'];
const possibleCards = baseCards.concat(baseCards); // Duplicate array items to make pairs
// Global Variables
const numCards = possibleCards.length;
const maxMatch = baseCards.length;      // Maximum Pairs
let opened = [];
let numStars = 3;
let numMatch = 0;
let numMoves = 0;
let turns = 0;
let isMatch = false;
let clickedCardName = '';           // Name of the card clicked
let clickedCardPosition = [];       // Position of the card clicked  
// When agent/robot helps
let hintCards = [];
let timeHint = 3500;
let speechFinished = false;
// Game timer 
let seconds = 0;
let minutes = 0;
let t;
// Pair timer
let myMinutes = 0;
let mySeconds = 0;
let myT;
// For communication with Server
var sessionId = document.getElementById('session-data-id').dataset.sessionId;
var language = document.getElementById('session-data').dataset.sessionLan;
const shuffleDiv = document.getElementById('session-data-shuffle');
const shuffleBoard = shuffleDiv.getAttribute('data-session-shuffle') == 'True'
let id_player = sessionId
let socket_address = ''
const url = ''
// reset cards
let allCardNames = [];                   // All cards names of new board
let consecutiveUnsuccessfulAttempts = 0; // moves
let trailsBeforeShuffle = 2;             // After how many moves should the board be reset? Only the first shuffle, then it will be modified (check cardClick)
let boardChanging = false;
// Congrats Message
const finishImg = ['walrus', 'penguin', 'tiger'];
const finishMsg = ['Oh man... even a walrus can do better', 'Good job, pal! Well done', 'Geez, That\'s amazing!'];
// Panel stars
const showStar = [
    '<li><i class="fa fa-star"></i></li><li><i class="fa fa-star-o"></i></li><li><i class="fa fa-star-o"></i></li>', // 1 star
    '<li><i class="fa fa-star"></i></li><li><i class="fa fa-star"></i></li><li><i class="fa fa-star-o"></i></li>', // 2 stars
    '<li><i class="fa fa-star"></i></li><li><i class="fa fa-star"></i></li><li><i class="fa fa-star"></i></li>' // 3 stars
];

/** ******************************************************************************************************************
 *                                                      GAME                                                         *                                               
 * ******************************************************************************************************************* 
 * */ 

/**
 * Start the game when page is loaded.
 */
document.addEventListener('DOMContentLoaded', initializeGame);

/**
 * Initialize the game
 */
function initializeGame() {
    socket_address = 'robot_hint_' + id_player
    console.log("beginning addr", socket_address)
    hintReceivedByRobot(socket_address)

    //document.querySelector('.overlay').style.display = 'none';
    document.querySelector('.deck').innerHTML = '';
    
    //alert(window.innerWidth);
    
    resetGameVariables();
    resetTimer();
    runTimer();
    myResetTimer();
    myRunTimer();
    printStars();
    printMoves();
    printTrials();
    changeLanguage()

    createCards();
}

/**
 * Reset variables when game is finished.
 */
function resetGameVariables() {
    opened = [];
    numStars = 3;
    numMoves = 0;
    numMatch = 0;
    turns = 0;
    isMatch = false;
}


/**
 * Shuffle the array of cards
 * @param {*} array 
 * @returns 
 */
function shuffle(array) {
    return array.sort(() => Math.random() - 0.5);
}

/**
 * Create deck
 */
function createCards() {
    const shuffledCards = shuffle(possibleCards);
    sendFlask("matrix", shuffledCards, '/game_board');

    shuffledCards.forEach((card, index) => {
        const cardElement = document.createElement('li');
        cardElement.classList.add('card');
        cardElement.id = index;
        cardElement.innerHTML = `<img src="/static/images/${card}.svg"/>`;
        cardElement.setAttribute('data-name', card);
        cardElement.addEventListener('click', () => cardClickListener(cardElement, card));
        document.querySelector('.deck').appendChild(cardElement);
    });
}

/**
 * Handle card clicks
 */
function cardClickListener(cardElement, card) {
    // if pair is already found the card of pair can't be clicked
    // or the game board is changing
    if (cardElement.classList.contains('match') || boardChanging) {
        return;
    }

    card = cardElement.getAttribute('data-name');

    setTimeout(() => {
        document.querySelectorAll(".card").forEach(card => {
            card.classList.remove('hint');
            card.classList.remove('flipInY');
            document.querySelector('.speech-bubble').style.display = 'none';
        });

        if (cardElement.classList.contains('show')) {
            return;
        }

        cardElement.classList.add('show', 'animated', 'flipInY');
        opened.push(card);
        console.log("Opened: " + opened)

        const filename = card.replace(/^.*[\\\/]/, '');
        clickedCardName = filename.replace(/\..+$/, '');

        const positionCard = Number(cardElement.id);
        const indexRow = Math.floor(positionCard / 6);
        const indexCol = positionCard % 6;

        clickedCardPosition = [indexRow, indexCol];

        if (opened.length > 1) {
            if (card === opened[0]) {
                match();
                consecutiveUnsuccessfulAttempts=0;
            } else {
                console.log("Shuffle board is " + shuffleBoard)
                // if shuffle is True check if the board should be changed
                if(shuffleBoard == true){
                    // after 6 turn the counter will be updated
                    if(turns > 6)
                        consecutiveUnsuccessfulAttempts++;
                    console.log("Trials: " + consecutiveUnsuccessfulAttempts)
                    unmatch();
                    if (trailsBeforeShuffle == consecutiveUnsuccessfulAttempts) {
                        printTrials();
                        trailsBeforeShuffle = 4;
                        consecutiveUnsuccessfulAttempts = 0 // reset since the board will change
                        boardChanging = true;
                        starCount();
                        printMoves();
                        setTimeout(() => {
                            changeBoard();
                            printTrials();
                        }, 750);
                        console.log("New shuffle")
                        return;
                    } 
                }
            }
        } else {
            isMatch = false;
        }

        starCount();
        printMoves();
        printTrials();

        if(numMatch === maxMatch ) {
            stopTimer();
            congrats();
        }

        // update turns number
        turns++;

        sendFlask("game", {
            "open_card_name": clickedCardName,
            "position": clickedCardPosition,
            "pairs": numMatch,
            "turn": turns,
            "match": isMatch,
            "n_face_up": opened.length,
            "time_until_match": `${myMinutes}:${mySeconds}`,
            "time_game": `${minutes}:${seconds}`,
            "board_changed": false,
            "new_board": allCardNames
        }, "/player_move");

        if (isMatch) {
            myMinutes = 0;
            mySeconds = 0;
        }
    }, 0);
}


/**
 * When the user finds a pair, the pair will not be covered.
 */
function match() {
    numMoves++;
    numMatch++;
    isMatch = true;
    opened = [];

    document.querySelectorAll(".show").forEach(matchedCard => {
        matchedCard.classList.add('match', 'animated', 'flip');
        matchedCard.classList.remove('show');
    });
}

/**
 * When user has not find a pair show error and increase moves.
 */
function unmatch() {
    numMoves++;
    isMatch = false;
    opened = [];

    document.querySelectorAll(".show:not(.match)").forEach(unmatchedCard => {
        unmatchedCard.classList = 'card show unmatch animated shake';
        document.querySelectorAll('.unmatch').forEach(unmatchedCard => {
            setTimeout(() => {
                unmatchedCard.classList = 'animated flipInY card';
            }, 600);
        });
    });
}

/**
 * Calculate Stars by the moves and print it
 */
function starCount() {
    if (numMoves <= 20) {
        numStars = 3;
    } else if (numMoves <= 27) {
        numStars = 2;
    } else {
        numStars = 1;
    }
    printStars();
}

// Print "stars", "moves", "matches" to the page
function printStars() {
    document.querySelectorAll('.stars').forEach(panel => panel.innerHTML = showStar[numStars - 1]);
}

function printMoves() {
    document.querySelectorAll('.moves').forEach(move => move.innerHTML = `<b>${numMoves}</b>`);
}

function printTrials() {
    document.querySelectorAll('.trials').forEach(move => move.innerHTML = `<b>${consecutiveUnsuccessfulAttempts}</b>`);
}

// Timer functions
function twoDigits(number) {
    return (number < 10 ? '0' : '') + number;
}

function myTimer() {
    mySeconds++
    if (mySeconds >= 60) {
        mySeconds = 0;
        myMinutes++;
    }
    myRunTimer();
}

function myRunTimer() {
    myT = setTimeout(myTimer, 1000);
}

function timer() {
    seconds++;
    if (seconds >= 60) {
        seconds = 0;
        minutes++;
    }

    updateTimer();
    runTimer();
}

function runTimer() {
    t = setTimeout(timer, 1000);
}

function resetTimer() {
    stopTimer();
    seconds = 0;
    minutes = 0;
    updateTimer();
}

function myResetTimer() {
    myStopTimer();
    mySeconds = 0;
    myMinutes = 0;
}

function updateTimer() {
    document.querySelectorAll(".timer-seconds").forEach(item => item.textContent = twoDigits(seconds));
    document.querySelectorAll(".timer-minutes").forEach(item => item.textContent = twoDigits(minutes));
}

function stopTimer() {
    clearTimeout(t);
}

function myStopTimer() {
    clearTimeout(myT)
}

/** ******************************************************************************************************************
 *                                           ANIMATION BWHEN OARD CHANGES                                            *                                               
 * ******************************************************************************************************************* 
 * */ 

function changeBoard(){
    /* Once the user has done many consecutive attempts the board game will change */
    stopTimer();
    myStopTimer();

    changeBoardPopUp();
    printTrials();
    setTimeout(() => {
        hideBoardPopup();
    }, 500);

    setTimeout(() => {
        shuffleUnmatchedCards();
    }, 1500);
}

// Pop-up when the robot provide a suggestion
function changeBoardPopUp() {
    const popup = document.getElementById('blur-popup');
    const title = popup.querySelector('.popup-title');
    const desc = popup.querySelector('.popup-description');

    if (language === 'en') {
        title.textContent = 'Oh no! The board game is changing!';
        desc.textContent = 'Please wait while the new board is being generated.';
    } else {
        title.textContent = 'Oh no! Il tabellone sta cambiando!';
        desc.textContent = 'Attendi un momento mentre il nuovo tabellone viene generato.';
    }

    setTimeout(() => {
        popup.style.display = 'flex';
    }, 10);
    
}

// Hide pop-up
function hideBoardPopup() {
    const popup = document.getElementById('blur-popup');
    const card = popup.querySelector('.popup-card');

    // after 1250 ms the animation will go
    setTimeout(() => {
        card.style.transition = 'opacity 0.4s ease';
        card.style.opacity = '0';

        popup.style.backgroundColor = 'transparent';
        popup.style.backdropFilter = 'none';

        setTimeout(() => {
            popup.style.display = 'none';

            card.style.opacity = '1';
            popup.style.backgroundColor = 'rgba(0, 0, 0, 0.6)';
            popup.style.backdropFilter = 'blur(8px)';
        }, 200); // transition animation time
    }, 1250); // time of popup
}


// shuffle un-matched cards and send to flask the last move and new board
function shuffleUnmatchedCards() {
    const deck = document.querySelector('.deck');
    const allCards = Array.from(deck.querySelectorAll('.card'));
    const unmatchedCards = allCards.filter(card => !card.classList.contains('match'));
    const matchedCards = allCards.filter(card => card.classList.contains('match'));

    const deckRect = deck.getBoundingClientRect();

    document.querySelectorAll(".card").forEach(card => {
        card.classList.remove('hint');
        card.classList.remove('flipInY');
        document.querySelector('.speech-bubble').style.display = 'none';
    });

    // Get center of grid
    const centerX = deckRect.width / 2;
    const centerY = deckRect.height / 2;

    // 0: hide pairs already found
    matchedCards.forEach(card => {
        card.classList.add('matched-hidden');
    });

    // 1. Animation towards the center of the board
    unmatchedCards.forEach(card => {
        const cardRect = card.getBoundingClientRect();
        const offsetX = centerX - (cardRect.left - deckRect.left + cardRect.width / 2);
        const offsetY = centerY - (cardRect.top - deckRect.top + cardRect.height / 2);

        card.style.setProperty('--center-x', `${offsetX}px`);
        card.style.setProperty('--center-y', `${offsetY}px`);

        // add first animation
        card.classList.add('shuffle-start', 'trail');
    });

    // Shuffle cards 
    requestAnimationFrame(() => {
        setTimeout(() => {
            // get the images of unmatched cards
            const unmatchedImages = unmatchedCards.map(card => {
                const img = card.querySelector('img');
                return img.getAttribute('src');
            });
            // shuffle them
            const shuffledImages = shuffle(unmatchedImages);
            // get the name of each card
            const unmatchedNames = shuffledImages.map(imgPath => {
                const filename = imgPath.replace(/^.*[\\\/]/, '');
                return filename.replace(/\..+$/, '');
            });

            // update unmatched cards (name and new index)
            unmatchedCards.forEach((card, index) => {
                const img = card.querySelector('img');
                img.setAttribute('src', shuffledImages[index]);

                const newName = shuffledImages[index].replace(/^.*[\\\/]/, '').replace(/\..+$/, '');
                card.setAttribute('data-name', newName);

                // hide cards
                card.classList.remove('show', 'animated', 'flipInY');
            });

            // the cards have been shuffled -> add the second animation
            unmatchedCards.forEach(card => {
                card.classList.remove('shuffle-start');
                card.classList.add('shuffle-end');
            });

            // Combines the pairs not found and those already found so that you have the whole board
            allCardNames = allCards.map(card => card.getAttribute('data-name'));

            // remove animations and send the new board to flask
            setTimeout(() => {
                unmatchedCards.forEach(card => {
                    card.classList.remove('shuffle-end', 'trail');
                });
                
                matchedCards.forEach(card => {
                    card.classList.remove('matched-hidden');
                });

                console.log("All new cards: " + allCardNames)
                console.log("New cards: " + unmatchedNames)

                // update turns number
                turns++;

                // if board is changed do not send the move just done
                console.log("Board changed? " + boardChanging)
                if(boardChanging) {
                    console.log("Board changed. This is new deck: " + allCardNames)
                    sendFlask("game", {
                        "open_card_name": clickedCardName,
                        "position": clickedCardPosition,
                        "pairs": numMatch,
                        "turn": turns,
                        "match": isMatch,
                        "n_face_up": opened.length,
                        "time_until_match": `${myMinutes}:${mySeconds}`,
                        "time_game": `${minutes}:${seconds}`,
                        "board_changed": true,
                        "new_board": allCardNames
                        }, "/player_move"
                    );
                }
                boardChanging = false;
            }, 850); // total time between start -> end
            runTimer();
            myRunTimer();
        }, 1000); // Waiting time for movement to the center
    });
}

/** ******************************************************************************************************************
 *                                                     POP-UP                                                        *                                               
 * ******************************************************************************************************************* 
 * */ 

/**
 * Set a pop-up when robot provide a hint.
 * In this way the user will look to the robot and they will not be able to click any card.
 */
function lookRobotPopup() {
    stopTimer();
    myStopTimer();

    const popup = document.getElementById('suggestion-popup');
    const title = popup.querySelector('.popup-title');
    const desc = popup.querySelector('.popup-description');

    if (language === 'en') {
        title.textContent = 'Hey, the robot is about to say something!';
        desc.textContent = "Remember that you can decide to not follow robot's suggestions";
    } 

    setTimeout(() => {
        popup.style.display = 'flex';
    }, 10);
}

/**
 * Hide pop-up when robot has finished to speak.
 */
function hidePopup() {
    const popup = document.getElementById('suggestion-popup');
    const card = popup.querySelector('.popup-cardHint');

    setTimeout(() => {
        card.style.transition = 'opacity 0.4s ease';
        card.style.opacity = '0';

        popup.style.backgroundColor = 'transparent';
        popup.style.backdropFilter = 'none';

        setTimeout(() => {
            popup.style.display = 'none';

            card.style.opacity = '1';
            popup.style.backgroundColor = 'rgba(0, 0, 0, 0.6)';
            popup.style.backdropFilter = 'blur(8px)';
        }, 200); // transition animation time

        stopTimer();
        myStopTimer();

        runTimer();
        myRunTimer();
    }, 500); // time of popup
}

/**
 * When the game is finished it shows a pop-up/form 
 * where the user can see the result of the game and answer the questions in the form.
 */
function congrats() {
    stopTimer();
    myStopTimer()
    const popup = document.getElementById('congrats-popup');

    // html elements to update
    const title = popup.querySelector('label[for="formText"] h1');
    const image = popup.querySelector('#congrats-image');

    
    // msg and image based on the number of stars
    title.textContent = finishMsg[numStars - 1] + '!'; 
    image.src = `static/images/${finishImg[numStars - 1]}.svg`;

    // show the pop-up
    setTimeout(() => {
        popup.style.display = 'flex';
    }, 500);
};


/** ******************************************************************************************************************
 *                                                     HINT                                                          *                                               
 * ******************************************************************************************************************* 
 * */ 

/**
 * This function makes sure that the suggestion is highlighted and 
 * that the suggestion is also written in the robot's speech-bubble.
 */
function hintReceivedByRobot(msg) {
    const socket = io.connect(url);

    socket.on(socket_address, handleRobotHintEvent);
    socket.on('Speech', handleSpeechEvent);
    socket.on('Feedback', handleFeedbackEvent);
    
    // Once the robot has finished uttering the suggestion, remove the pop-up 
    function handleSpeechEvent(msg) {
        speechFinished = true;
        hidePopup();
    }

    // if robot will provide a feedback, an alert will be shown (using robot icon on panel)
    function handleFeedbackEvent(msg) {
        console.log("feedback");

        // get message for speech-bubble
        if(language == 'italiano') 
            msg = '<span class="hint-text"> Sto per parlare! </span>';
        else 
            msg = 'span class="hint-text"> I\'m about to speak! </span>';

        // Show message near to robot icon if app is multithread
        const speechBubble = document.querySelector('.speech-bubble');
        speechBubble.innerHTML = msg;
        speechBubble.style.display = 'block';

        // Hide the speech bubble after 2.5 seconds
        setTimeout(() => {
            speechBubble.style.display = 'none';
        }, 2500);
    }
    
    function handleRobotHintEvent(msg) {
        const obj = JSON.parse(msg);
        const isRobotConnected = obj.action.isRobotConnected;

        if (isRobotConnected != false) {
            lookRobotPopup();
        } else {
            timeHint = 0;
        }

        setTimeout(applyHint, timeHint);

        function applyHint(){    
            const suggestion = obj.action.suggestion;
            const row = obj.action.position[0] - 1
            const col = obj.action.position[1] - 1

            console.log("suggestion", suggestion, row, col)

            // get message for speech-bubble
            msg = getMessageText(suggestion, row, col);

            // Show message near to robot icon if app is multithread
            document.querySelector('.speech-bubble').innerHTML = msg
            document.querySelector('.speech-bubble').style.display = 'block';

            // Highlight suggestion
            document.querySelectorAll(".card").forEach((card) => {
                if(card.classList.contains("flipInY") == true) //&& (turns + 1) % 2 != 0)
                    card.classList.remove('flipInY')

                if(card.classList.contains("match") == false && card.classList.contains("flipInY") == false 
                                                            && card.classList.contains("show") == false){
                    if(suggestion == "row"){
                        if(row == Math.floor(card.id/6))
                            card.classList.add('hint');
                    }

                    if(suggestion == "column"){
                        if(col == card.id % 6)
                            card.classList.add('hint');
                        }
                    }

                    if(suggestion == "card"){
                        if(row == Math.floor(card.id/6) && (col == card.id % 6))
                            card.classList.add('hint');
                    }
            });
        }
    }
}

/**
 * This function will return the sentence (ita or english, based on the chosen language)
 * that will be showed in the speech-bubble
 */
function getMessageText(suggestion, row, col){
    if(language == 'italiano'){
        let textHint = '';
        if (suggestion == "row") 
            textHint = '<span class="hint-text">Prova la riga ' + (row + 1) + '!</span>';
        else if (suggestion == "column")
            textHint = '<span class="hint-text">Prova la colonna ' + (col + 1) + '!</span>';
        else
            textHint = '<span class="hint-text">Prova in riga ' + (row + 1) + ' e colonna ' + (col + 1) + '!</span>';

        return textHint
    } else {
        // inglese
        let textHint = '';
        if (suggestion == "row") 
            textHint = '<span class="hint-text">Try the row ' + (row + 1) + '!</span>';
        else if (suggestion == "column")
            textHint = '<span class="hint-text">Try the column ' + (col + 1) + '!</span>';
        else
            textHint = '<span class="hint-text">Try the card in row ' + (row + 1) + ' and column ' + (col + 1) + '!</span>';

        return textHint
    }
}

/** ******************************************************************************************************************
 *                                                     FLASK                                                         *                                               
 * ******************************************************************************************************************* 
 * */ 

// Send data to Flask server
function sendFlask(flag, data, route){
    console.log("Request by", route)
    fetch(route + "/" + id_player, {
        headers : {
            'Content-Type': 'application/json',
            'Accept': 'application/json'
        },
        method : 'POST',
        body : JSON.stringify( {
          [flag]: data
        })
    })
    .then(response => response.json())
    .then(data => {
        console.log("Data received", data)
    })
    .catch(function(error) {
        console.log(error);
    });  
}

/** ******************************************************************************************************************
 *                                                     UTILS                                                         *                                               
 * ******************************************************************************************************************* 
 * */ 

/**
 * If game is not finished and user try to refresh the page
 */
window.addEventListener('beforeunload', function (e) {
    if (!(numMatch == 12)) {
        var text = ''
        if(language == 'inglese')
            text = 'Are you sure you want to refresh the page? The game is not finished yet.'
        else
            text = 'Sei sicuro di voler aggiornare la pagina? Il gioco non è ancora finito.'
        var confirmationMessage = text;

        (e || window.event).returnValue = confirmationMessage;
        console.log("winwod", confirmationMessage)
        return confirmationMessage;
    } else {
        console.log("Page refreshed...")
    }
});

/**
 * If the user has reloaded the page anyway, send to server this information to clear csv and create a new file for them.
 */
window.addEventListener('unload', function(event) {
    if (!(numMatch == 12)) {
        console.log('Un cheattone ha ricaricato la pagina nonostate il gioco non fosse finito.');
        fetch('/cheating/' + id_player)
                .then(response => response.text())
                .then(data => {console.log(data);})
                .catch(error => console.error('Error:', error));
        //window.location.href = '/game';
    }
});

// Check if it's the first visit
function checkFirstVisit() {
    if (document.cookie.indexOf('mycookie') === -1) {
        document.cookie = 'mycookie=1';
        console.log("First visit...")
    } else {
      //  sendFlask("refreshed", "True", "/");
      console.log("Not first visit...")
    }
}

/**
 * Once the user press "exit", Flask will show the home page
 */
document.getElementById('congrats-popup').querySelector('.exit').addEventListener('click', function() {
    console.log("User has pressed exit!")
    fetch('/exit', {
        method: 'GET', 
        headers: {
            'Content-Type': 'application/json'
        },
    })
        .then(response => {
            console.log("Redirect to home page!");
            window.location.href = '/index';
        })
        .catch(error => console.error('Error:', error));
});

function changeLanguage(){
    if(language == 'inglese'){
        // label
        document.querySelector('label[for="formText2"]').textContent = "Thanks for playing!"
    }
}