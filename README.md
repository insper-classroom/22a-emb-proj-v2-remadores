# Projeto Embarcados

Desenvolvendo um controle remoto.

## Entrega 1 <img src="https://img.shields.io/static/v1?label=Entrega1&message=Finalizado&color=success&style=flat-square&logo=ghost"/>

## Integrantes

:sassy_man: Henrique Martinelli Frezzatti;

:sassy_woman: Lívia Sayuri Makuta;

## Ideia :thought_balloon: :video_game: :joystick:

A ideia que tivemos foi a de fazer um controle de videogame no estilo do `Playstation 4`.  Esse controle teria duas entradas analógicas e quatro entradas digitais.
As estradas analógicas seriam dois `Joypad's` que seriam usados para:

&nbsp; &nbsp; &nbsp; :heavy_check_mark: A movimentação do personagem no jogo;

&nbsp; &nbsp; &nbsp; :heavy_check_mark: A movimentação da câmera do personagem no jogo.

Além disso, no controle também teríamos 4 entradas digitais que serim 4 botões, cada um representaria as seguintes funções no controle:

&nbsp; &nbsp; &nbsp;  :x: O X;

&nbsp; &nbsp; &nbsp;  :o:	 A bola;

&nbsp; &nbsp; &nbsp;  :small_red_triangle: O triângulo;

&nbsp; &nbsp; &nbsp;  :red_square: O quadrado.

E esse controle poderia ser utilizado para jogos de `Playstation 4` jogados no Windows.

## Nome 	:copyright:

**Suny SP4**

## Usuários :bust_in_silhouette: :bust_in_silhouette:

Os usuários do nosso controle possivelmente seriam:

&nbsp; &nbsp; &nbsp; :heavy_check_mark: Jogadores de videogame

&nbsp; &nbsp; &nbsp; :heavy_check_mark: Game Testers

Observação: isso não exclui a possibilidade de outras pessoas o utilizarem.

## Software/Jogo :desktop_computer: :chess_pawn:

O nosso controle irá controlar como software os jogos de `Playstation 4`/ `Xbox` que podem ser emulados no Windows através do `Xbox Game Pass`. E a ideia inicial é a de testar com jogos que são principalmente de tiro : `FPS` (do inglês *First Person Shooter*).

## Jornada do usuários (3 pts) :repeat:


![Jornada](https://user-images.githubusercontent.com/62647438/160206110-29e15dce-e0c3-4fe6-8966-094f8ace4467.png)


Nosso usuário é um estudante que tem uma rotina corrida **(I)** e que durante o tempo livre ama jogar videogame **(II)**. 

Entretanto, nem sempre ele está em casa e como gosta tanto da experiência imersiva de jogar com um controle **(III)**, resolveu adquirir um principalmente por se tornar algo portátil e que ele pode usar em qualquer lugar com seu computador **(IV)**, ou seja, ele pode emular seus jogos favoritos em seu computador **(V)** e jogar com seu controle em qualquer lugar, especialmente durante as pausas que faz durante o seu dia **(VI)**. E como o controle é bem intuitivo e parecido com o de seu console favorito, sua experiência se aproxima a de jogar em um `Playstation 4` mesmo que esteja jogando em um computador. 

E para usar o controle basta: 

&nbsp; &nbsp; &nbsp; :heavy_check_mark: Ligá-lo;

&nbsp; &nbsp; &nbsp; :heavy_check_mark: Fazer pareamento com o computador;

&nbsp; &nbsp; &nbsp; :heavy_check_mark: Começar a jogar.

## Comandos/ Feedbacks (2 pts) 	:memo::back:

Os comandos possíveis de nosso controle são:

&nbsp; &nbsp; &nbsp; :heavy_check_mark: Andar para todas as direções.

&nbsp; &nbsp; &nbsp; :heavy_check_mark: Todos os comandos relacionados ao X, Quadrado, Triângulo e Bola de um jogo comum. 

&nbsp; &nbsp; &nbsp; :heavy_check_mark: Mexer a câmera com o `Joypad's` em um jogo 3D.

Além disso, o usuário irá receber os seguintes feedbacks em relação aos comandos:

&nbsp; &nbsp; &nbsp; :heavy_check_mark: Quando o controle estiver ligado uma luz verde estará acesa.

&nbsp; &nbsp; &nbsp; :heavy_check_mark: Quando o personagem estiver andando, uma luz estará piscando. 

&nbsp; &nbsp; &nbsp; :heavy_check_mark: Quando algum dos botões for pressionado, uma luz irá piscar e apagar conforme o click do botão.

## In/OUT (3 pts) :arrow_left::arrow_right:

### ENTRADAS:

&nbsp; &nbsp; &nbsp; :heavy_check_mark: Movimentação do personagem: `Joypad` esquerdo (analógica)

&nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;  :black_small_square:	 Racional: Como a movimentação do personagem se dá nas três direções: x, y e z, o `Joypad` resolve. Assim, se o usuário direcionar o personagem para a direita, ele começará a se mover para a direita. 

&nbsp; &nbsp; &nbsp; :heavy_check_mark: Movimento da câmera do personagem: `Joypad' Direito (analógica)

&nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; :black_small_square: Racional: Como a movimentação da câmera do personagem se dá igualmente nas três direções: x, y e z, o `Joypad` também resolve. Logo, se o usuário fizer um giro com o `Joypad` ele terá um visão 360 do que está à sua volta.

&nbsp; &nbsp; &nbsp; :heavy_check_mark: 4 botões: X, Quadrado, Triângulo e Bola. (digital)

&nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; :black_small_square: Racional: Assim como em um controle de videogame tradicional, os botões :x:, :o:, 🔺 e 🟥 irão executar ações quando pressionados, seja ela de soltar uma arma, ou jogar uma granada, entre outras. Porém essas ações mudam de jogo para jogo. O que faremos é atrelar o click de um botão ao comando equivalente no computador. 


### SAÍDAS:
&nbsp; &nbsp; &nbsp; :heavy_check_mark: Luz piscando de maneira a identificar quando o personagem estiver andando. (dispositivo digital)

&nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; :black_small_square:	Racional: Queremos fornecer ao usuário um feedback de que seu personagem está se movimentando através de um meio visual, que além de ser moderno traz um charme e é algo apreciado por muitos gamers. Sendo que essa fita de LED pode ser colocada ao redor do controle. 


&nbsp; &nbsp; &nbsp; :heavy_check_mark: Luz ligada para demonstrar que o controle está ligado. (digital)

&nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; :black_small_square: Racional: Queremos que o usuário saiba que o controle está ligado e foi conectado ao computador por meio de uma luz, ou seja, antes de começar a jogar ele saberá se teve algum problema com o dispositivo ou se está tudo certo para poder começar a jogar. Assim, visualmente, se um pequena luz verde for ligada, ele sabe que está tudo "pronto" para começar a se divertir. 


&nbsp; &nbsp; &nbsp; :heavy_check_mark: Luz pisca e apaga conforme o click do botão.

&nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; :black_small_square:	Racional: Queremos mais uma vez mostrar ao usuário que sua ação está sendo recebida pelo controle, ou seja, quando ele clicar em algum dos botões, ele irá ver que de fato o botão foi pressionado pois um LED irá acender e apagar conforme o tempo do click do usuário.

## Design (2 pts) :pushpin::triangular_ruler:

Esboço de como seria o controle (detalharemos melhor isso em uma próxima etapa):


![Prototipo](https://user-images.githubusercontent.com/62647438/160212921-0a505798-483a-4319-9f75-e4fe6ecadd06.jpeg)


Esboço feito no Fusion 360 de como queríamos que o controle fosse (se não tivéssemos que nos preocupar com fios, entre outras coisas).

Vista lateral:
![26ce53f2-c857-42b2-881e-c1327be141db](https://user-images.githubusercontent.com/62647438/166697801-a2f9260f-a905-48e3-af26-35c78cc03c7d.PNG)

Vista de frente:
![0eba9784-81c3-49e2-b2be-6e8af4a871f0](https://user-images.githubusercontent.com/62647438/166697836-fe767ec7-1862-430b-add6-52dc61501268.PNG)

Além disso, o esboço pode ser acessado no seguinte link: https://a360.co/3OX8W6P .
