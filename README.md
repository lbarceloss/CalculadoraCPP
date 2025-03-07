# Calculadora C++ com Node.js
Este projeto é um aplicativo de console em C++ que utiliza Node.js para realizar cálculos do PangYa. A lógica matemática da calculadora foi originalmente escrita em JavaScript para uso em um ambiente web e, neste projeto, foi adaptada para ser executada no backend via Node.js.

A comunicação entre C++ e Node.js é feita por meio de chamadas de sistema, onde o programa C++ envia os dados para um script Node.js, que processa os cálculos e retorna os resultados. Esse método permite aproveitar a flexibilidade e dinamismo do JavaScript para cálculos complexos, mantendo a robustez e eficiência do C++.

O projeto pode ser expandido para suportar diferentes tipos de operações matemáticas e integração com outras aplicações.

## Baixando e instalando dependencia necessarias.
1. Instale o Git e o Node.js
2. Baixe e instale as ferramentas nos sites oficiais:
* [Git](https://git-scm.com/)
* [Node.js](https://nodejs.org/en)

4. Instale a biblioteca "curl" seguindo estes passos:
* Primeiro, abra o Git Bash ou o Prompt de Comando e clone o repositório oficial do cURL:
  
``` 
git clone https://github.com/curl/curl.git
```
* Enseguida execute o comando:
```
git clone https://github.com/microsoft/vcpkg.git
cd vcpkg
.\bootstrap-vcpkg.bat
```
* Por fim:
```
.\vcpkg.exe install curl:x64-windows
```
## Configurar o Visual Studio 2022 para usar o cURL

Abra o Projeto no Visual Studio 2022

### Adicionar a Biblioteca ao Projeto

* No Solution Explorer, clique com o botão direito no projeto e selecione Properties (Propriedades).
  
* Vá até C/C++ → General → Additional Include Directories e adicione o caminho da pasta include do vcpkg:
```
C:\path\to\vcpkg\installed\x64-windows\include
```
### Adicionar os Arquivos de Biblioteca
* Em Linker → General → Additional Library Directories, adicione:
```
C:\path\to\vcpkg\installed\x64-windows\lib
```
* Em Linker → Input → Additional Dependencies, adicione:
```
libcurl.lib
```
## Utilizando

* Abra o seu Prompt de Comando e digite o seguinte comando:
 
```
node server.js
```
* Depois execute o programa no Visual Studio.
