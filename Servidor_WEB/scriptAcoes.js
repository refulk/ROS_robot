//executa após carregar a janela HTML
window.onload = function() 
{
	// loadCmbCarga();	
	// console.log("OK2");
 //    refreshImgVideo();
 	exibirDados();
	window.setInterval(function() {
		location.reload(true);
	}, 1000);
};	

function exibirDados()
{
    document.getElementById("tabDados").innerHTML = ""; //Clean Table
    var tabela = document.getElementById('tabDados');      
    var acumulaHistorico = "";  
    if(dados.length == 1)
    {
        var linha = tabela.insertRow(0);
        var cel_nome = linha.insertCell(0);
        cel_nome.innerHTML = "> Sem dados";             
    }
    else
    {
        for (var j=0; j < dados.length - 1; j++)
        {     
            acumulaHistorico += dados[j] + " ";     
        }
        var linha = tabela.insertRow(-1);
        var cel_nome = linha.insertCell(0);
        cel_nome.innerHTML = ""; 

        linha = tabela.insertRow(-1);
        cel_nome = linha.insertCell(0);
        cel_nome.innerHTML = acumulaHistorico;   
    }

}